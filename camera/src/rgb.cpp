// https://github.com/IntelRealSense/librealsense/blob/master/doc/stepbystep/getting_started_with_openCV.md

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp> 
#include <fstream>
#include <vector>
using namespace std;

#define MINI(a,b) ((a) < (b) ? (a) : (b))

int height, width, frame_rate;
image_transport::Publisher pub;
sensor_msgs::ImagePtr msg;
int min_value=10000;
void imageCallbackRGB(const sensor_msgs::ImageConstPtr& imsg)
{
  //ROS_INFO("Image Parameters: width[%d] height[%d] isbigEnd[%d] enc[%s] step[%d]",imsg->width,imsg->height,imsg->is_bigendian,imsg->encoding.c_str(),imsg->step);
  int img_val;
  int sz=imsg->data.size();
  vector<unsigned char> dummy(sz);
  int cnt=0;
  //imsg->width has to be even
  for(int i=0;i<imsg->height;i++){
    for(int j=0;j<imsg->step;j++){
      dummy[cnt]=(unsigned char)(imsg->data[i*imsg->step+j]);
      cnt+=1;
    }
  }
  unsigned char *p = &*(dummy.begin());
  cv::Mat color(cv::Size(imsg->width, imsg->height), CV_8UC3, (void*)p, cv::Mat::AUTO_STEP);
  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();
  pub.publish(msg);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "color_stream");
  ros::NodeHandle n;

  n.param("/color_stream/frame_rate", frame_rate, 30);
  n.param("/color_stream/resolution_height",height, 480);
  n.param("/color_stream/resolution_width", width, 848);
  ROS_INFO("Color parameters: %d, %d, %d", frame_rate, height, width);

  image_transport::ImageTransport it(n);
  pub = it.advertise("camera/image", 1000);
  ros::Subscriber sub = n.subscribe("camera/color/image_raw", 1000, imageCallbackRGB);
  ros::spin();
  return 0;
}
