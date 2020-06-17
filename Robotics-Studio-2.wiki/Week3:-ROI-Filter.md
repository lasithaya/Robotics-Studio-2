If you look at the images captured by the Duckibot camera the lane lines are at the lower half of the image. We don't need to process the top half of the image to detect lanes. In this tutorial, we will create a Region of Interest (ROI) filter that crop the top half of the image so we can focus on the bottom half of the image. 

Create `example6_ROI_filter.py` file in `\nodes` and add the `region_of_interest` function as follows

```python
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
```

This function first creates an all-zero image and then fills the pixels on the bottom half of the image with  255. This image mask is then applied to the original edge-filtered image by `bitwise_and` operation to filter-out the bottom half of the image.

Run the node and you will see an image similar to the following image.

![](https://miro.medium.com/max/320/1*w4gv5_H581ayXxVoQHR6Wg.png)

```python
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
        edge_filtered = detect_edges(orig)
        roi_filtered = region_of_interest(edge_filtered)
    except Exception as err:
        print err
    # show results
    show_image(roi_filtered)
    

    
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
        
        
def show_image(img):
    cv2.imshow('ROI Filter', img)
    cv2.waitKey(1)

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
```

