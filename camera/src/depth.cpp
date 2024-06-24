#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <algorithm>
#include <sensor_msgs/Image.h>
#include <camera/Depth.h>
#include<vector>
using namespace std;

int depth_width, depth_height, frame_rate;
int width_dim;
int height_dim;
int width_dim_LR;
int height_dim_LR;
int cx;
int cy;

float getAverageDepth(const sensor_msgs::ImageConstPtr& msg, float width, float height, int x, int y) {
  float sum = 0;
  int ctr = 0;
  //ROS_INFO("Depth Parameters: width[%d] height[%d] isbigEnd[%d] step[%d]",msg->width,msg->height,msg->is_bigendian,msg->step);
  //ROS_INFO("SEQ [%d]",msg->header.seq);
  for(int i = y; i < y + height; i++){
    for (int j = x; j/2 < x + width; j+=2) {
      float depth_val = ((msg->data[(i*msg->step)+(j+1)])<<8)|(msg->data[(i*msg->step)+(j)]);
      if (depth_val > 0) {
        ctr += 1;
        // ROS_INFO("Depth Center DV[%f] CNT[%d]",depth_val,ctr);
        sum += depth_val;
      }
    }
  }
  if (ctr == 0) {
    return 0;
  }
  // ROS_INFO("Depth Center [%lf]",(sum/ctr));
  return (sum / ctr);
}

float getAverageDepthLR(const sensor_msgs::ImageConstPtr& msg, float width, float height, int x, int y) {
  float sum = 0;
  int ctr = 0;
  for(int i = y; i < y + height; i++) {
    for (int j = x; j/2 < x + width; j+=2) {
      float depth_val = ((msg->data[(i*msg->step)+(j+1)])<<8)|(msg->data[(i*msg->step)+(j)]);
      if (depth_val > 0) {
        ctr += 1;     
        sum += depth_val;
      }
    }
  }
  if (ctr == 0) {
    return 0;
  }
  // ROS_INFO("Depth LR [%lf]",(sum/ctr));
  return (sum / ctr);
}

int getMedian(const sensor_msgs::ImageConstPtr& msg, float width, float height) {
  int start = height/2 - 5;
  int end = height/2 + 5;
  int len = end -start + 1;
  vector<vector<float>> temp(width,vector<float>(len)); //temp[COL][ROW]
  vector<float> arr(width);
  float depth_val;
  for(int i = start; i < end; i++) {
    for (int j = 0; j/2 < width; j+=2) {
      depth_val = ((msg->data[(i*msg->step)+(j+1)])<<8)|(msg->data[(i*msg->step)+(j)]);
      // ROS_INFO("row[%d] [%d]",j/2,i-start);
      temp[j/2][i-start]=depth_val;
    }
  }
  for(int i=0;i<width;i++)
  {
    sort(temp[i].begin(),temp[i].end());
    arr[i]=temp[i][len/2];
  }
  float max = 0;
  for(int i=0; i<width;i++){
    if(arr[i]>max){
      max = arr[i];
    }
  }
  return max;
}

float* getCorners(float width, float height, int cx, int cy) {
  float *corners = new float[6];
  // TODO: need to check what the right value is, with 1280 * 960, we used 20, avoiding noisy edges
  corners[0] = 40; // Left Image
  corners[1] = (cy - height / 2); // Left Image
  corners[2] = (cx - width / 2); // Central Image. cx-50 paul.
  corners[3] = (cy - height / 2); //Central Image; height=50 radius of 50 around cy. 
  // TODO: need to check what the right value is, with 1280 * 960, we used 20
  corners[4] = (depth_width - width_dim_LR - 20); // Right Image.
  corners[5] = (cy - height / 2); // Right Image.
  return corners;
}

camera::Depth dmsg;
ros::Publisher depth_pub;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  float* corners = getCorners(width_dim, height_dim, cx, cy);
  // TBD remove first 30 frames probably use seq_id.
  dmsg.left_depth = getAverageDepthLR(msg, width_dim_LR, height_dim_LR, corners[0], corners[1]);
  dmsg.center_depth = getAverageDepth(msg, width_dim, height_dim, corners[2], corners[3]);
  dmsg.right_depth = getAverageDepthLR(msg, width_dim_LR, height_dim_LR, corners[4], corners[5]);
  dmsg.di = getMedian(msg, depth_width, depth_height);
  depth_pub.publish(dmsg);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "depth_stream");
  ros::NodeHandle n;
  // Default Parameters (to be used for race)
  n.param("/depth_stream/frame_rate", frame_rate, 60);
  n.param("/depth_stream/resolution_height", depth_height, 480);
  n.param("/depth_stream/resolution_width", depth_width, 848);
  depth_pub = n.advertise<camera::Depth>("camera/depth", 1000);
  ROS_INFO("Depth Parameters: %d, %d, %d", frame_rate, depth_height, depth_width);
  width_dim = 100;
  height_dim = 100;
  width_dim_LR = 50;
  height_dim_LR = 50;
  cx = depth_width / 2;
  cy = depth_height / 2;
  ros::Subscriber sub = n.subscribe("camera/depth/image_rect_raw", 1000, imageCallback);
  ros::spin();
  return 0;
}