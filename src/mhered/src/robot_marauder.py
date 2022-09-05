#!/usr/bin/env python3

"""
**Marauder Robot**

Assignment 2 Part 2: PID controller

Proportional controller of angular and linear speed such as the robot moves
 smoothly without hitting obstacles.

* Make linear velocity proportional to the fwd distance to obstacles - similar
  to Go-to-Goal Behavior

* Make angular velocity proportional to the distance to obstacles on the left
  and right sides: rotate smoothly to the right if obstacles on the left are
  much closer than on the right (or to the left in the opposite case)

The robot should move forever without hitting obstacles.
Tested on Turtlebot3 in the maze and house environments.

"""


import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

# use current location from the global variables
# (constantly updated by odom_callback())
global x, y, yaw, fwd_clearance
x = 0.0
y = 0.0
yaw = 0.0
fwd_clearance = 0.0
left_clearance = 0.0
right_clearance = 0.0

# get global params
global LASER_RANGE, BEAM_ANGLE
global ANGLE_TOL, SAFETY_DIST, MIN_CLEARANCE
global WAIT, LIN_SPEED, ROT_SPEED, RATE


def odom_callback(odom_message):
    """
    Constantly extract robot pose from /odom nav_msgs/Odometry
    """

    # update global variables
    global x, y, yaw

    x = odom_message.pose.pose.position.x
    y = odom_message.pose.pose.position.y
    q = odom_message.pose.pose.orientation
    q_as_list = [q.x, q.y, q.z, q.w]
    (_, _, yaw) = euler_from_quaternion(q_as_list)


def scan_callback(message):
    """
    Constantly update global fwd_clearance from /scan sensor_msgs/LaserScan
    """

    global fwd_clearance, right_clearance, left_clearance

    # parameters
    global BEAM_ANGLE, LASER_RANGE

    ranges = np.asarray(message.ranges)
    angles = np.degrees(
        (message.angle_min + message.angle_increment * np.asarray(range(len(ranges))))
    )

    # shift angles >180 from [180 - 360) to [-180, 0)
    angles_shifted = np.where(angles > 180, angles - 360, angles)

    angles_indeces = angles_shifted.argsort()
    angles_sorted = angles_shifted[angles_indeces[::-1]]
    ranges_sorted = ranges[angles_indeces[::-1]]

    # remove Inf and NaN values from ydata to compute axes limits
    # deep copy to avoid modifying ydata
    clean_ranges_sorted = ranges_sorted.copy()
    clean_ranges_sorted[~np.isfinite(clean_ranges_sorted)] = LASER_RANGE

    # slice the beam at -BEAM_ANGLE +BEAM_ANGLE
    fwd_ranges = [
        r for (a, r) in zip(angles_sorted, clean_ranges_sorted) if abs(a) < BEAM_ANGLE
    ]
    fwd_clearance = min(fwd_ranges)

    right_ranges = [
        r for (a, r) in zip(angles_sorted, clean_ranges_sorted) if (a > 30 and a < 100)
    ]
    right_clearance = min(right_ranges)

    left_ranges = [
        r
        for (a, r) in zip(angles_sorted, clean_ranges_sorted)
        if (a < -30 and a > -100)
    ]
    left_clearance = min(left_ranges)

    print(
        f"\nCLEARANCE FWD: {fwd_clearance:8.4}"
        f" LEFT: {left_clearance:8.4}"
        f" RIGHT: {right_clearance:8.4}\n"
    )


def marauder_robot(velocity_publisher):
    """Marauder robot based on Go to goal method"""

    # use current location from the global variables
    # (constantly updated by odom_callback())
    global x, y, yaw
    # use current clearances from the global variables
    # (constantly updated by scan_callback())
    global fwd_clearance, left_clearance, right_clearance

    # get global params
    global LASER_RANGE, BEAM_ANGLE
    global WAIT, RATE
    global THRESHOLD, K_DISTANCE, K_ANGLE, SAFETY_DIST

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # publish the velocity at RATE Hz (RATE times per second)
    loop_rate = rospy.Rate(RATE)

    while True:
        vel_lin = vel_ang = 0
        if fwd_clearance > SAFETY_DIST:
            vel_lin = K_DISTANCE * (fwd_clearance - SAFETY_DIST)

        if abs(right_clearance - left_clearance) > THRESHOLD:
            vel_ang = K_ANGLE * (right_clearance - left_clearance)

        velocity_message.linear.x = vel_lin
        velocity_message.angular.z = vel_ang

        # THRESHOLD here to avoid fluctuations?
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()


if __name__ == "__main__":
    try:

        # declare the node
        rospy.init_node("marauder_node", anonymous=True)

        # declare velocity publisher
        cmd_vel_topic = "/cmd_vel"
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        # declare /odom subscriber
        odom_topic = "/odom"
        odom_subscriber = rospy.Subscriber(odom_topic, Odometry, odom_callback)

        # declare /scan subscriber
        scan_topic = "/scan"
        scan_subscriber = rospy.Subscriber(scan_topic, LaserScan, scan_callback)

        time.sleep(2.0)

        # get ROS parameters (or default)
        global LASER_RANGE, BEAM_ANGLE
        global WAIT, RATE
        global THRESHOLD, K_DISTANCE, K_ANGLE, SAFETY_DIST

        LASER_RANGE = rospy.get_param("LASER_RANGE", 5.0)  # m
        BEAM_ANGLE = rospy.get_param("BEAM_ANGLE", 5.0)  # degrees

        WAIT = rospy.get_param("WAIT", 0.5)  # s
        RATE = rospy.get_param("RATE", 10.0)  # Hz

        THRESHOLD = rospy.get_param("THRESHOLD", 0.15)  # m/s
        K_DISTANCE = rospy.get_param("K_DISTANCE", 0.25)  # 1/s
        K_ANGLE = rospy.get_param("K_ANGLE", 10.0)  # 1/s
        SAFETY_DIST = rospy.get_param("SAFETY_DIST", 0.6)  # m

        # launch the marauder robot app
        marauder_robot(velocity_publisher)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
