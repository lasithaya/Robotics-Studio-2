To implement the lane controller of our robot we are subscribing to the `/duckiebot/camera_node/image/compressed` topic. This topic publishes compressed images (lightweight) from the camera and therefore the frame rate of this topic is high. If you use the `rqt_image_viewer` you would see a continuous video stream. We are not using the `cv_bridge` to convert the `compressed images` to the OpenCV format, but use the following code to convert them.

```python
from sensor_msgs.msg import CompressedImage

`
`
`
np_arr = np.fromstring(msg.data, np.uint8)
orig = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

```

 