#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np



def start_node():
    rospy.init_node('image_subcriber')
    rospy.loginfo('image_subcriber node started')
    rospy.Subscriber("image", Image, process_image)
    rospy.spin()
    
def process_image(msg):
    try:
       # convert sensor_msgs/Image to OpenCV Image
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        filtered=detect_edges(orig)
    except Exception as err:
        print err
    # show results
    show_image(filtered)
    

    
def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([60, 40, 40])
    upper_blue = np.array([150, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    

    # detect edges
    edges = cv2.Canny(mask, 200, 400)

    return edges
    
    
        
        
def show_image(img):
    cv2.imshow('Edge detection', img)
    cv2.waitKey(1)

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
