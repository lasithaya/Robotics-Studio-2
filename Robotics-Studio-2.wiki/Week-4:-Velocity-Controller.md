The final part of our lane controller code is to navigate the robot inside the two lanes line. We will use the heading angle w.r.t to lanes to steer the robot to the center of the two lanes. If the heading is positive we will move the robot to the right and vise versa if the heading is negative.  This can become quite tricky to fine-tune as there are some parameters that need to adjust according to your lane environment and the robot. It also depends on the frame rate of the received camera images and their latency. 

To execute velocity commands,  we need to create a  publisher to the `/cmd_vel` topic of the robot and send out forward velocity and angular velocities. The following command creates a publisher object with the ROS Twist message type. 

```python
from geometry_msgs.msg import Twist
vel_pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)

```

We will be using a simple proportional controller to adjust the heading of the robot.  The following code implements the proportional controller to adjust the heading. The two proportional controller values `kp_two_lines` and `kp_single_line` are the two parameters you need to adjust if your robot is failing to move within lanes

```python
# proportional controller values 
    kp_two_lines = -0.015
    kp_single_line= 0.005  
    
    if num_lane==2:
        forward_vel=0.1*vel_mul
        angular_vel= kp_two_line*float(orientation)
        
    elif num_lane==1:
        forward_vel=0.02 * vel_mul
        angular_vel=kp_single_line*float(orientation)
    else  :
        forward_vel=0.00
        angular_vel=0.00
```

The forward velocity needs to be controlled according to the desired steering angle. If the required steering angle is high then we need to slow down the robot and if the steering angle is near zero we will move the robot with the desired forward velocity. In the following code, the forward velocity goes to zero if the steering angle is more than 30 degrees.

```python
vel_reducer=(1-abs(orientation)/30.0)      
    if  vel_reducer < 0.0:
        vel_mul=0.0        
    else:
        vel_mul=vel_reducer 


```

Finally, the following function publishes the required velocity commands.

```python
def publishing_vel( forward_vel, angular_vel):
    vel = Twist()
    vel.angular.x = 0.0
    vel.angular.y = 0.0
    vel.angular.z = angular_vel
    vel.linear.x = forward_vel
    vel.linear.y = 0.0
    vel.linear.z = 0.0
    vel_pub.publish(vel)     



```





