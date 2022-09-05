# Assignment 2

Questions for this assignment

1. Find the topic name of the pose (position and orientation) of turtlesim and  its message type. Display the content of message of the pose.

Name: `/turtle1/pose`

Type:  `turtlesim/Pose`

Content: 

```
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```

2. Find the topic name of the velocity command of turtlesim and its message  type. Display the content of message of the velocity command. Remember that velocity command is the topic that makes the robot move.

Name: `/turtle1/cmd_vel`

Type:  `geometry_msgs/Twist`

Content: 

```
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

Write a simple ROS program called turtlesim_pose.py, which subscribes to the  topic of the pose, and then prints the position of the robot in the callback function. We provide you a program with missing code that you need to complete. 

```python
#!/usr/bin/env python

import rospy

# task 1. import the Pose type from the module turtlesim
from turtlesim.msg import Pose


def pose_callback(message):  
	# task 4. display the x, y, and theta received from the message
	print("Pose callback")
	print(f"x = {message.x:10.2}") 
	print(f"y = {message.y:10.2}") 
	print(f"yaw = {message.theta:10.2}")

    
if __name__ == '__main__':
	try:
		rospy.init_node('turtlesim_motion_pose', anonymous=True)
		
		# task 2. subscribe to the topic of the pose of the Turtlesim
		
		rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
		
        # task 3. spin
		rospy.spin()

	except rospy.ROSInterruptException:
		rospy.loginfo("Node terminated.")
```



Note: need to run `chmod +x /home/mhered/catkin_ws/src/ros_essentials_cpp/src/my_assignment/my_turtle_pose.py` to make python file executable!

complete the previous code to add a publisher to the velocity and make the robot move for a certain distance.

Complete the missing code in this following program

```python
#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import math
import time

from std_srvs.srv import Empty

x=0
y=0
z=0
yaw=0

def pose_callback(message):
    global x
    global y, z, yaw
    x = message.x
    y = message.y
    yaw = message.theta
    # print(f"Pose (x, y, yaw): ({x:10.3}, {y:10.3}, {yaw:10.3})")
    
def move_turtle(speed, distance):
    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # get current location from the global variable before entering the loop 
    x0 = x
    y0 = y
    # z0=z
    # yaw0=yaw     

    # task 1. assign the x coordinate of linear velocity to the speed.
    velocity_message.linear.x = speed
    velocity_message.linear.y = 0.0
    velocity_message.linear.z = 0.0
    velocity_message.angular.x = 0.0
    velocity_message.angular.y = 0.0
    velocity_message.angular.z = 0.0
    

    distance_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times per second)

    # task 2. create a publisher for the velocity message on the appropriate topic.

    # what am I missing??

    while True:
        rospy.loginfo("Turtlesim moves forward")

        # task 3. publish the velocity message
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        
        # rospy.Duration(1.0)
        # measure the distance moved
        distance_moved = distance_moved + abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print(distance_moved)        
        if not (distance_moved < distance):
            rospy.loginfo("reached")
            break
        
    # task 4. publish a velocity message zero to make the robot stop after the distance is reached**
    velocity_message.linear.x = 0.0
    velocity_message.linear.y = 0.0
    velocity_message.linear.z = 0.0
    velocity_message.angular.x = 0.0
    velocity_message.angular.y = 0.0
    velocity_message.angular.z = 0.0

    velocity_publisher.publish(velocity_message)

if __name__ == '__main__':
    try:
        rospy.init_node('my_turtle_pose_node', anonymous=True)
        
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, pose_callback) 
    
        # task 5. declare velocity publisher
        velocity_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(velocity_topic, Twist, queue_size=10)
    
        time.sleep(1)
        print('move: ')

        # call to the function
        move_turtle(1.0, 5.0)
        
        time.sleep(2)
        print('start reset: ')
        rospy.wait_for_service('reset')
        reset_turtle = rospy.ServiceProxy('reset', Empty)
        reset_turtle()
        print('end reset: ')
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")

```



Note: need to run `chmod +x /home/mhered/catkin_ws/src/ros_essentials_cpp/src/my_assignment/my_turtle_pose2.py` to make python file executable!
