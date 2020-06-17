### Create Image subscriber and  Image-Processing Node
The next node will subscribe to the `image` topic and display it in a window using OpenCV library functions

 1. As before, create a basic ROS python node (`example2_image_subcriber.py`) and set its executable permissions:

    ```python
    #!/usr/bin/env python
    import rospy
    

    def start_node():
        rospy.init_node('image_subcriber')
        rospy.loginfo('image_subcriber node started')

    if __name__ == '__main__':
        try:
            start_node()
        except rospy.ROSInterruptException:
            pass
    ```

    ```bash
    chmod u+x nodes/example2_image_subcriber.py
    ```

    * Note that we don't have to edit `CMakeLists` to create new build rules for each script, since python does not need to be compiled.

 1. Add a ROS subscriber to the `image` topic, to provide the source for images to process.

    1. Import the `Image` message header

       ```python
       from sensor_msgs.msg import Image
       ```

    1. Above the `start_node` function, create an empty callback (`process_image`) that will be called when a new Image message is received:

       ```python
       def process_image(msg):
           try:
              pass
           except Exception as err:
               print err
       ```
       * The try/except error handling will allow our code to continue running, even if there are errors during the processing pipeline.

    1. In the `start_node` function, create a ROS Subscriber object:
       * subscribe to the `image` topic, monitoring messages of type `Image`
       * register the callback function we defined above

       ```python
       rospy.Subscriber("image", Image, process_image)
       rospy.spin()
       ```

       * reference: [rospy.Subscriber](http://docs.ros.org/melodic/api/rospy/html/rospy.topics.Subscriber-class.html)
       * reference: [rospy.spin](http://docs.ros.org/melodic/api/rospy/html/rospy-module.html#spin)

    1. Run the new node and verify that it is subscribing to the topic as expected:

       ```bash
       rosrun my_cv_tutorial example2_image_subcriber.py
       rosnode info /image_subcriber
       rqt_graph
       ```

 1. Convert the incoming `Image` message to an OpenCV `Image` object and display it
 As before, we'll use the `CvBridge` module to do the conversion.

    1. Import the `CvBridge` modules and create  `CvBridge` object

       ```python
       from cv_bridge import CvBridge
       bridge = CvBridge()
       ```

    1. In the `process_image` callback, add a call to the CvBridge [imgmsg_to_cv2](https://docs.ros.org/api/cv_bridge/html/python/) method:

       ```python
       # convert sensor_msgs/Image to OpenCV Image
       orig = bridge.imgmsg_to_cv2(msg, "bgr8")
       ```
       * This code (and all other image-processing code) should go inside the `try` block, to ensure that processing errors don't crash the node.
       * This should replace the placeholder `pass` command placed in the `try` block earlier

    1. Use the OpenCV `imshow` method to display the images received.  We'll create a pattern that can be re-used to show the result of each image-processing step.

       1. Import the OpenCV `cv2` module:

          ```python
          import cv2
          ```

       1. Add a display helper function above the `process_image` callback:

          ```python
          def show_image(img):
              cv2.imshow('image', img)
              cv2.waitKey(1)
          ```

      

       1. **Below** the `except` block (outside its scope; at `process_image` scope, display the `drawImg` variable:

          ```python
          # show results
          show_image(orig)
          ```

    1. Run the node and see the received image displayed.
    

### Complete code
```python
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
```


