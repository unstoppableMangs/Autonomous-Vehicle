#!/usr/bin/python
import cv2, rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool 
from cv_bridge import CvBridge
import pytesseract



class detect_image_node(object):
    def __init__(self):
        self.stop_sign_pub = rospy.Publisher('/stop_sign_found',Bool, queue_size=10)
        self.bridge = CvBridge()
        self.mx_iter = 10000
        rospy.Subscriber('camera/image',Image,self.image_callback)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        cv_image = cv2.resize(cv_image,(848,480))
        stop_sign_cascade = cv2.CascadeClassifier("stopgpt_classifier.xml")
        gray = cv2.cvtColor(stop_sign_image, cv2.COLOR_BGR2GRAY)
        stop_signs = stop_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        if len(stop_signs) > 0:
            rospy.loginfo("Stop sign FOUND")
            self.stop_sign_pub.publish(True)
        else:
            self.stop_sign_pub.publish(False)
    
if __name__=='__main__':
    rospy.init_node('see_image')
    detect_image_node = detect_image_node()
    rospy.spin()
