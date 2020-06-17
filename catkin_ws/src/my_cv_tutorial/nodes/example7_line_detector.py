#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

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
        edge_filtered = detect_edges(orig)
        roi_filtered = region_of_interest(edge_filtered)
       #line detection 
        lane_lines = detect_line_segments(roi_filtered)
       
       #draw detected lines on the origial image 
        lane_lines_image = display_lines(orig, lane_lines)
        
    except Exception as err:
        print err
    # show results
    show_image(lane_lines_image)
    

    
def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([60, 40, 40])
    upper_blue = np.array([150, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    

    # detect edges
    edges = cv2.Canny(mask, 200, 400)

    return edges
    

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus bottom half of the screen
    polygon = np.array([[
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (0, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges
    
    
def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=8, maxLineGap=4)

    return line_segments
    
    
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

        
        
def show_image(img):
    cv2.imshow('ROI Filter', img)
    cv2.waitKey(1)

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
        
