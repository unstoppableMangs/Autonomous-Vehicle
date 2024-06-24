import cv2
import numpy as np


stop_sign_image = cv2.imread("many.jpg")
stop_sign_cascade = cv2.CascadeClassifier("stopgpt_classifier.xml")


gray = cv2.cvtColor(stop_sign_image, cv2.COLOR_BGR2GRAY)


stop_signs = stop_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

for (x, y, w, h) in stop_signs:
    print(x,y,w,h)
