# Assignment 7

## The Move-Stop-Rotate Behavior for Obstacle Avoidance

We will first start with a very simple program to make the robot move around 

1. First, make sure to install Turtlebot 3 Robot, as shown in the Section  "Getting Started with Turtlebot3". It is recommended to use the latest  version of ROS, which is ROS Noetic, although it should work on any  version. 
2. Start the Turtlebot3 Robot in the maze environment `roslaunch turtlebot3_gazebo turtlebot3_world.launch`
3. Start RVIZ for Visualization `roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch`
4. Observe the laser scanner in red dots in RVIZ. You can check the thickness of  the laser scanner by changing the Size (m) attribute in rviz to 0.04  instead of 0.01. This will provide better visualization of the laser  scanner topic. 
5. Run the `teleop` node `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`. Make the robot move and observe how the laser scanner layout is changing in `rviz`.
6. Now, develop a ROS program in Python AND in C++ to make the robot move in a straight line, and stop when the closest obstacle gets closer than 0.6 meters. Use a laser beam around the center of  [-5,+5] degrees. 
7. Then, develop another function that  makes the robot rotate until the straight distance to obstacles in the same beam aperture is greater than 3 meters. 
8. Make a loop to repeat these two functions and observe if the robot succeeds in moving without hitting the obstacle forever or not. 
9. Try the same program in the house environment `roslaunch turtlebot3_gazebo turtlebot3_house.launch`. What are your observations?

## A PID Controller

The motion behavior in the previous solution is not smooth as there are several motions interrupted by a stop.

The objective of this second assignment is to develop a Proportional  controller that regulates the angular speed and the linear speed such as the robot moves smoothly without hitting obstacles. 

- The linear velocity can be adjusted to be proportional to the distance (similar to what we did in Go-to-Goal Behavior)

- The angular velocity can be adjusted based on the distance to the obstacles on the left side and the distance to obstacles on the right side. 

- If the distance to the left side obstacle is much closer than the right  side obstacles, then make the robot rotate smoothly to the right.

- If the distance to the right side obstacle is much closer than the left  side obstacles, then make the robot rotate smoothly to the left.

- Test the code on Turtlebot3 in the maze environment and house environment. 

- Test it on a real robot if you have one (any kind of real robot).

  

## 15/08/22

* create `bumper.launch` to  launch environment: `turtlebot3`, `rviz`, `teleop`, and `scan_subscriber`
* fix `scan_subscriber.py` : sort data to plot `/scan` in -180 180 degree range and fix bug with shallow copy of `clean_ydata`

## 16/08/22

* add `bumper_add.py` to implement the Move-Stop-Rotate Behavior for Obstacle Avoidance 
* subscribe to `/odom`to get robot pose
* subscribe to `/scan` get distance to object in beam +/-5 degrees, dealing with NaN - `global fwd_clearance`
* publish `cmd_vel` commands to move the robot `global x, y, yaw`
* Loop:
  * behavior 1 - move straight to goal until obstacle closer than 0.6m
  * behavior 2 - rotate until direction with clearance >3m found 

## 17/08/22

* test and fine-tune `bumper_app.py` in **maze** and **house** environments

![bouncy_robot](assets/images/bouncy_robot.gif)

* Add `smooth_bumper_app.py` with first implementation of the PID controller based on go-to-goal behavior

## 18/08/22

* Update and rename `bumper_app.py` to [`robot_bouncy.py`](../src/mhered/src/robot_bouncy.py) 
* Update and rename `smooth_bumper_app.py` to [`robot_marauder.py`](../src/mhered/src/robot_marauder.py)
* Eliminate `smooth_bumper.launch`
* Rename `bumper.launch` to [`robot.launch`](../src/mhered/launch/robot.launch) and modify to take parameters:
  * `robot_behavior:= <bouncy>, marauder, instructor, student1, student2`
  * `world:= <world>, house`
  * `open_rviz:= true , <false>`
  * `plot_laser:= true , <false>`

```bash
$ roslaunch mhered robot.launch robot_behavior:=instructor plot_laser:=true
```

* update this .md file

* test alternative implementations:
  * instructor solution: [robot_instructor.py](../src/mhered/src/robot_marauder.py) - ok but got stuck
  * PID implementation by student 1 ([Mohammed Zia Ahmed Khan](https://www.udemy.com/user/mohammed-zia-ahmed-khan-2/)) : [robot_student1.py](../src/mhered/src/robot_student1.py) - behaves as a slow bouncy, quite good, no collision
  * PID implementation by student  2 ([Wellington Noberto](https://www.udemy.com/user/wellington-noberto/)) : [robot_student2.py](../src/mhered/src/robot_student2.py) - ok but skids and got stuck