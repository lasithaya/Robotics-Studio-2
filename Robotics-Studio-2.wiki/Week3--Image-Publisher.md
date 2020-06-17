We will be using OpenCV image processing libraries for lane detection in this tutorial.

### 1. Install OpenCV in ROS

The following packages are required to be installed to complete this tutorial. Install them according to your  ROS version. The example below is for ROS Kinetic.

`sudo apt-get install ros-kinetic-cv-bridge`
 
`sudo apt-get install ros-kinetic-image-view`

### 2. Create a  ROS Package
We are going to create a ROS python package for this tutorial. Inside the `YOUR_CATKIN_WS/src` create the following package with associated dependencies.

`catkin_create_pkg my_cv_tutorial sensor_msgs cv_bridge rospy std_msgs`

* all ROS packages depend on `rospy`
    *  we'll use `cv_bridge` to convert between ROS's standard Image message and OpenCV's Image object
    * `cv_bridge` also automatically brings in dependencies on the relevant OpenCV modules

Create a python module for this package:

```bash
cd my_cv_tutorial
mkdir nodes
```
### 3.Create an Image Publisher
The first node will read in an image from a file and publish it as a ROS [Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) message on the `image` topic.

 * Note: ROS already contains an `image_publisher` package/node that performs this function, but we will duplicate it here to learn about ROS Publishers in Python.

1. Create a new python script for our image-publisher node (`nodes/example1_image_publisher.py`).  Fill in the following template for a skeleton ROS python node:

    ```python
    #!/usr/bin/env python
    import rospy

    def start_node():
        rospy.init_node('image_pub')
        rospy.loginfo('image_pub node started')

    if __name__ == '__main__':
        try:
            start_node()
        except rospy.ROSInterruptException:
            pass
    ```

 1. Allow execution of the new script file:

    ```bash
    chmod u+x nodes/example1_image_publisher.py
    ```

 1. Build the package and run the image publisher:

    ```bash
    catkin_make
    roscore
    rosrun my_cv_tutorial example1_image_publisher.py
    ```

    * You should see the "node started" message

 1. Read the image file to publish, using the filename provided on the command line

    1. Import the `sys` and `cv2` (OpenCV) modules:

       ```python
       import sys
       import cv2
       ```

    1. Pass the command-line argument into the `start_node` function:

       ```python
       def start_node(filename):
       ...
       start_node( rospy.myargv(argv=sys.argv)[1] )
       ```
       * Note the use of `rospy.myargv()` to strip out any ROS-specific command-line arguments.

    1. In the `start_node` function, call the OpenCV [imread](https://docs.opencv.org/3.0-beta/modules/imgcodecs/doc/reading_and_writing_images.html#imread) function to read the image.  Then use [imshow](https://docs.opencv.org/3.0-beta/modules/highgui/doc/user_interface.html#imshow) to display it:

       ```python
       img = cv2.imread(filename)
       cv2.imshow("image", img)
       cv2.waitKey(2000)
       ```

    1. Run the node, with the specified image file:

       ```bash
       rosrun my_cv_tutorial example1_image_publisher.py ~/road.jpeg
       ```
       * You should see the image displayed
       * Comment out the `imshow`/`waitKey` lines, as we won't need those any more
       * Note that you don't need to run `catkin build` after editing the python file, since no compile step is needed.
1. Convert the image from OpenCV Image object to ROS Image message:

    1. Import the `CvBridge` and `Image` (ROS message) modules and create `CvBridge` object :

       ```python
       from cv_bridge import CvBridge
       from sensor_msgs.msg import Image
       bridge = CvBridge()
       ```

    1. Add a call to the CvBridge [cv2_to_imgmsg](https://docs.ros.org/api/cv_bridge/html/python/) method:

       ```python       
       imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")
       ```

1. Create a ROS publisher to continually publish the Image message on the `image` topic.  Use a loop with a 1 Hz throttle to publish the message.

    ```python
    pub = rospy.Publisher('image', Image, queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(imgMsg)
        rospy.Rate(1.0).sleep()  # 1 Hz
    ```

1. Run the node and inspect the newly-published image message
    1. Run the node (as before):

       ```bash
       rosrun my_cv_tutorial example1_image_publisher.py ~/road.jpeg
       ```

    1. Inspect the message topic using command-line tools:

       ```bash
       rostopic list
       rostopic hz /image
       rosnode info /image_pub
       ```

    1. Inspect the published image using the standalone [image_view](http://wiki.ros.org/image_view#image_view.2BAC8-diamondback.image_view) node

       ```bash
       rosrun image_view image_view image:=/image
       ```
    
### Complete code
```python
#!/usr/bin/env python
import rospy
import sys
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
bridge = CvBridge()

def start_node(filename):
    rospy.init_node('image_pub')
    rospy.loginfo('image_pub node started')
    img = cv2.imread(filename)    
    imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")
    pub = rospy.Publisher('image', Image, queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(imgMsg)
        rospy.Rate(1.0).sleep()  # 1 Hz

if __name__ == '__main__':
    try:
        start_node( rospy.myargv(argv=sys.argv)[1] )
    except rospy.ROSInterruptException:
        pass
````
