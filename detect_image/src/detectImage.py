#!/usr/bin/python
import cv2, rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool 
from cv_bridge import CvBridge



class detect_image_node(object):
    def __init__(self):
        self.stop_sign_pub = rospy.Publisher('/stop_sign_found',Bool, queue_size=10)
        self.bridge = CvBridge()
        self.mx_iter = 10000
        rospy.Subscriber('camera/image',Image,self.image_callback)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        if cv_image is None:
            return
        cv_image = cv2.resize(cv_image,(253,200))#(848,480))
        stop_sign_cascade = cv2.CascadeClassifier("/home/odroid/catkin_ws_sc1/src/detect_image/src/stopgpt_classifier.xml")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        stop_signs = stop_sign_cascade.detectMultiScale(cv_image, scaleFactor=1.02, minNeighbors=2, minSize=(100,100))
        if len(stop_signs) > 0:
            rospy.loginfo("Stop sign FOUND")
            self.stop_sign_pub.publish(True)
        else:
            rospy.loginfo("Stop sign NOT FOUND")
            self.stop_sign_pub.publish(False)
    
if __name__=='__main__':
    rospy.init_node('see_image')
    detect_image_node = detect_image_node()
    rospy.spin()
