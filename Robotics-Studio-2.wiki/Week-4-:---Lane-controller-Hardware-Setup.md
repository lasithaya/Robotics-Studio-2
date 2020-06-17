In this tutorial, we will develop a lane controller for our robot.

### Hardware Setup
It is important that we fix a right-handed coordinate system for our robot.  It should be as shown in the following figure with  X-axis in the forward direction, Y-axis to the left and Z-axis to up ([More](https://docs.duckietown.org/DT19/learning_materials/out/duckiebot_modeling.html)). When positive velocity command is given, it should move forward and positive rotational velocity should rotate the robot in the anti-clockwise direction (Viewed from top). You can use the `teleop_twist_keyboard` ([`teleop_twist_keyboard`][http://wiki.ros.org/teleop_twist_keyboard))
package to control your robot by connecting to your robot remotely with your laptop. If your robot is behaving differently then you might have to swap the wire connection of your robot's motors.

![](https://docs.duckietown.org/DT19/learning_materials/out/assets/data-from-img-mod-kin-f7d13bca.png)(source :[Duckiebot Modeling](https://docs.duckietown.org/DT19/learning_materials/out/duckiebot_modeling.html))

### Network Setup
We are going to subscribe to the camera images from the robot and run our lane controller on your laptop. This way it's easy to debug and tune your code. You can follow the instructions provided earlier on how to set up remote connections to the robot.  I would highly recommend using a dedicated wireless router for your laptop-robot connection with the laptop connected to the router via an ethernet cable. This will minimize any latency issues and you should be able to view the video stream from the robot's camera in near realtime. This is important because the robot will react to the lane detected by the camera and any latency in the received camera images will cause absurd behavior with your robot.




