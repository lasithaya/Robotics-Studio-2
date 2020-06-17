In the ROI_filtered from the previous tutorial, we can clearly see four lines. In order to Duckiebot to navigate within these lanes, we need to find the exact coordinates of these lines.
OpenCV contains a  function, called Hough Transform, which does exactly this. Hough Transform is a technique used in image processing to extract features like lines, circles, and ellipses. We will use it to find straight lines from a bunch of pixels that seem to form a line. The function HoughLinesP essentially tries to fit many lines through all the white pixels and return the most likely set of lines, subject to certain minimum threshold constraints.

![](https://miro.medium.com/max/300/1*6pYR6ZliHiC0b9sAilJ1RQ.gif)

More information of openCV Hough Trasnform can be found here . 
[Hough Transform](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html)

Internally, HoughLineP detects lines using Polar Coordinates. Polar Coordinates (elevation angle and distance from the origin) is superior to Cartesian Coordinates (slope and intercept), as it can represent any lines, including vertical lines which Cartesian Coordinates cannot because the slope of a vertical line is infinity. HoughLineP takes a lot of parameters:

* **rho** is the distance precision in pixel. We will use one pixel.
* **angle** is angular precision in radian. 
* **min_threshold** is the number of votes needed to be considered a line segment. If a line has more votes, Hough Transform considers them to be more likely to have detected a line segment
minLineLength is the minimum length of the line segment in pixels. Hough Transform won’t return any line segments shorter than this minimum length.
* **maxLineGap** is the maximum in pixels that two line segments that can be separated and still be considered a single line segment. For example, if we had dashed lane markers, by specifying a reasonable max line gap, Hough Transform will consider the entire dashed lane line as one straight line, which is desirable.

**Setting these parameters is really a trial and error process**

Create `exmaple7_line_detector.py` in `\node` and add the following `detect_line_segments` function to detect line segments.

```python
def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=8, maxLineGap=4)

    return line_segments
```

We will also create a function to draw the line segments on the original image

```python
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image
```

Run the node and you will see a lane filtered image as follows
![](https://miro.medium.com/max/320/1*7j9nD-N39rYgvAKyNqwpeA.png)


### complete code

```python
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
        

```

