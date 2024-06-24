#! /usr/bin/env python
import rospy, time
from std_srvs.srv import Empty

if __name__ =='__main__':
    rospy.init_node('calibrate')
    time.sleep(5)
    calibrate = rospy.ServiceProxy('imu/calibrate', Empty)
    calibrate()
    
