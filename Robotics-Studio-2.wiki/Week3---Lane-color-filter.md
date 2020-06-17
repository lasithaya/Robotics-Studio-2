### Lane color filter

Now let's filter out the lane line based on its color from the converted HSV image.

create a new `example4_blue_filter.py` file in `\node`s folder and modify the process_image function to the following and create a new `blue_filter` function to process the image. Import `numpy` module by adding `import numpy as np`

```python
def process_image(msg):
    try:
       # convert sensor_msgs/Image to OpenCV Image
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        filtered=blue_filter(orig)
    except Exception as err:
        print err
    # show results
    show_image(filtered)
    
def blue_filter(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([60, 40, 40])
    upper_blue = np.array([150, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    return mask
```

Here we are using BGR to HSV transformation, not RBG to HSV. This is because OpenCV, for some legacy reasons, reads images into BGR (Blue/Green/Red) color space by default, instead of the more commonly used RGB (Red/Green/Blue) color space. They are essentially equivalent color spaces, just order of the colors swapped.

From the HSV image now we can filter out blue-colored areas. We can specify a range for Blue color in HSV space. 
In Hue color space, the blue color is in about 120–300 degrees range, on a 0–360 degrees scale. These ranges need to be found experimentally and you can also use a much tighter range in your case. 

![](https://miro.medium.com/max/360/1*j6ijyV8txxRl7lS9fvj6mw.png)

OpenCV uses a range of 0–180, instead of 0–360, so the blue range we need to specify in OpenCV is 60–150 (instead of 120–300). These are the first parameters of the lower and upper bound arrays. The second (Saturation) and third parameters (Value) are needed to find experimentally. The 40–255 ranges work reasonably well for both Saturation and Value in this case.

Run this node and it will display a filtered image.

![](https://miro.medium.com/max/320/1*qwTowxuvUbYQXyOZgQVjRw.png)

### Complete Code
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
        filtered=blue_filter(orig)
    except Exception as err:
        print err
    # show results
    show_image(filtered)
    
def blue_filter(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([60, 40, 40])
    upper_blue = np.array([150, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    return mask
        
        
def show_image(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
```
