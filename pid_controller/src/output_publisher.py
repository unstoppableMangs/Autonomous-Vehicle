#!/usr/bin/python
import rospy
from simple_pid import PID #https://pypi.org/project/simple-pid/
from std_msgs.msg import Float32
from pid_controller.msg import Depth
import numpy as np

class pid_node(object):
    def __init__(self):
        self.pid_pub = rospy.Publisher('/pid_output',Float32, queue_size=10)
        rospy.Subscriber("/camera/depth", Depth ,self.depth_callback)
        self.pid = PID(0.0009, 0, 0)
        self.left_depth_arr, self.right_depth_arr = [2000] * 5, [2000] * 5

    def depth_callback(self, data):
        l_measure = np.clip(data.left_depth, 500, 2000)
        r_measure = 2000 if data.right_depth > 2000 else data.right_depth
        r  = self.rolling_avg(self.right_depth_arr, r_measure)
        l = self.rolling_avg(self.left_depth_arr, l_measure)
        output = self.pid(l*.90-r)
        #rospy.loginfo("PID OUTPUT: %f", output)
        self.pid_pub.publish(output)
    
    def rolling_avg(self, arr, measurement):
        arr.append(measurement)
        arr.pop(0)
        return int(sum(arr)/len(arr))
        
if __name__ == "__main__":
    rospy.init_node('pid_node')
    pid_node = pid_node()
    rospy.spin()
