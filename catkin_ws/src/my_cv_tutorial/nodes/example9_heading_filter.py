#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import cv2
import math
import numpy as np


vel_pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
curr_orientation_angle=0.0
error_accumulator=0.0


def start_node():
    rospy.init_node('lane_controller')
    rospy.loginfo('image_subcriber node started')
    
    rospy.Subscriber("/duckiebot/camera_node/image/compressed", CompressedImage, process_image)
    
    rospy.spin()
    
def process_image(msg):
    try:
       #convert sensor_msgs/CompressedImage to OpenCV Image
       
       # orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        np_arr = np.fromstring(msg.data, np.uint8)
        orig = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        edge_filtered = detect_edges(orig)
        roi_filtered = region_of_interest(edge_filtered)
       #line detection 
        line_segments = detect_line_segments(roi_filtered)
        
        lane_lines = average_slope_intercept(orig, line_segments)
        #lane_lines = combine_lines (orig, line_segments)
       
       #draw detected lines on the origial image 
        lane_lines_image = display_lines(orig,lane_lines)
            
    except Exception as err:
        print err
        
        
    global curr_orientation_angle    
    robot_orientation_new=compute_heading_angle(orig, lane_lines)
    image_heading = display_heading_line( lane_lines_image,robot_orientation_new)
    curr_orientation_angle= curr_orientation_angle*0.0+ robot_orientation_new*1.0
    
    lane_controller( curr_orientation_angle,len(lane_lines))    
    # show results
    show_image(image_heading )
    
def lane_controller(orientation,num_lane):
    
    global error_accumulator
    vel_reducer=(1-orientation/30.0)      
    if  vel_reducer < 0.0:
        vel_mul=0.0        
    else:
        vel_mul=vel_reducer
        
    error_accumulator=error_accumulator+orientation    
    if  (error_accumulator) <-300.00:        
        error_accumulator= -300.00
    elif (error_accumulator) >300.00:
        error_accumulator= 300.00  
    
    rospy.loginfo('error accumulator %f', error_accumulator)
        
    # proprotional controller values 
    kp_two_lines = -0.005
    kp_single_line= 0.005 
    ki_two_lines=-0.0009
    
    if num_lane==2:
        forward_vel=0.01*vel_mul
        angular_vel= kp_two_lines*float(orientation) + ki_two_lines*error_accumulator
        
    elif num_lane==1:
        forward_vel=0.02 * vel_mul
        angular_vel=kp_single_line*float(orientation)
    else  :
        forward_vel=0.00
        angular_vel=0.00
    #forward_vel=0.0
    publishing_vel( forward_vel, angular_vel)
    
    
def publishing_vel( forward_vel, angular_vel):
    vel = Twist()
    vel.angular.x = 0.0
    vel.angular.y = 0.0
    vel.angular.z = angular_vel
    vel.linear.x = forward_vel
    vel.linear.y = 0.0
    vel.linear.z = 0.0
    vel_pub.publish(vel)     
    

    
def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([80, 140, 40])
    upper_blue = np.array([120, 255, 255])
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
    min_threshold = 50  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=5, maxLineGap=10)                                    
    return line_segments
    
    
def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        rospy.loginfo('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                #rospy.loginfo('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            
            if slope < 0:               
               left_fit.append((slope, intercept))
            else:
               right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))
    

    return lane_lines
    
    
    
def combine_lines (frame, line_segments):  
    color_1=[255, 0, 0] 
    color_2=[0, 0, 255] 
    thickness=2
    lane_lines = []
    if line_segments is None:
        rospy.loginfo('No line_segment segments detected')
        return lane_lines
    
    # state variables to keep track of most dominant segment
    largestLeftLineSize = 0
    largestRightLineSize = 0
    largestLeftLine = (0,0,0,0)
    largestRightLine = (0,0,0,0)
    left_lane_flag= False
    right_lane_flag= False
    
    for line in line_segments:
        for x1,y1,x2,y2 in line:
            size = math.hypot(x2 - x1, y2 - y1)
            slope = ((y2-y1)/(x2-x1))
            # Filter slope based on incline and
            # find the most dominent segment based on length
            if (slope > 0.0): #right
                if (size > largestRightLineSize):
                    largestRightLine = (x1, y1, x2, y2)
                    right_lane_flag= True                  
               #cv2.line(frame, (x1, y1), (x2, y2), color_1, thickness)
            elif (slope < 0.0): #left
                if (size > largestLeftLineSize):
                    largestLeftLine = (x1, y1, x2, y2)
                    left_lane_flag= True
               #cv2.line(frame, (x1, y1), (x2, y2), color_2, thickness)
                
    if  left_lane_flag: 
        x1,y1,x2,y2 = largestLeftLine         
        fit_left = np.polyfit((x1,y1), (x2,y2), 1) 
        cv2.line(frame, (x1, y1), (x2, y2), color_1, thickness)
        lane_lines.append(make_points(frame, fit_left))   
    
    if  right_lane_flag:
        x1,y1,x2,y2 = largestRightLine    
        fit_right = np.polyfit((x1,y1), (x2,y2), 1) 
        cv2.line(frame, (x1, y1), (x2, y2), color_2, thickness)  
        lane_lines.append(make_points(frame, fit_right))
    
    return lane_lines
    
def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]
    
    
def compute_heading_angle(frame, lane_lines):
    height, width, _ = frame.shape

    if len(lane_lines) == 2: # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0][0] # extract left x2 from lane_lines array
        _, _, right_x2, _ = lane_lines[1][0] # extract right x2 from lane_lines array
        mid = float(width / 2)
        x_offset = float( (left_x2 + right_x2) / 2 - mid)
        y_offset = float(height / 2)  
 
    elif len(lane_lines) == 1: # if only one line is detected
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = float(x2 - x1)
        y_offset = float(height / 2)

    elif len(lane_lines) == 0: # if no line is detected
        x_offset = 0.0
        y_offset = float(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = float(angle_to_mid_radian * 180.0 / math.pi)  
    heading_angle = angle_to_mid_deg 
    rospy.loginfo("Heading angle %f :",angle_to_mid_deg)

    
    return heading_angle  
    
   
    
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image
    
    
def display_heading_line(frame, heading_angle, line_color=(0, 0, 255), line_width=5 ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    
    heading_angle_radian = (heading_angle + 90.0) / 180.0 * math.pi 
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(heading_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

        
        
def show_image(img):
    cv2.imshow('ROI Filter', img)
    cv2.waitKey(1)

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
        
