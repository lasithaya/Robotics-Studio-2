#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()


def start_node():
    rospy.init_node('image_subcriber')
    rospy.loginfo('image_subcriber node started')
    rospy.Subscriber("image", Image, process_image)
    rospy.spin()
    
def process_image(msg):
    try:
       # convert sensor_msgs/Image to OpenCV Image
       orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        
    except Exception as err:
        print err
    # show results
    show_image(orig)
        
        
def show_image(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
