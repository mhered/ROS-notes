#!/usr/bin/env python3

"""
** BUG0 stub methods**
    Repeat:
        * move straight towards the goal until obstacle detected
        * follow obstacle boundary until no obstacle in direction of the goal
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
from sensor_msgs.msg import LaserScan

import math
import time


# use current location from the global variables
# (constantly updated by odom_callback())
global x, y, yaw
x = 0.0
y = 0.0
yaw = 0.0

# use clearances from the global variables
# (constantly updated by scan_callback())
global fwd_clearance, left_clearance, right_clearance
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

    # use clearances from the global variables
    # (constantly updated by scan_callback())
    global fwd_clearance, left_clearance, right_clearance

    # parameters
    global BEAM_ANGLE, LASER_RANGE

    ranges = np.asarray(message.ranges)
    angles = np.degrees((message.angle_min +
                         message.angle_increment * np.asarray(
                             range(len(ranges)))))

    # shift angles >180 from [180 - 360) to [-180, 0)
    angles_shifted = np.where(angles > 180, angles-360, angles)

    angles_indeces = angles_shifted.argsort()
    angles_sorted = angles_shifted[angles_indeces[::-1]]
    ranges_sorted = ranges[angles_indeces[::-1]]

    # remove Inf and NaN values from ydata to compute axes limits
    # deep copy to avoid modifying ydata
    clean_ranges_sorted = ranges_sorted.copy()
    clean_ranges_sorted[~np.isfinite(clean_ranges_sorted)] = LASER_RANGE

    # slice the beam at -BEAM_ANGLE +BEAM_ANGLE
    fwd_ranges = [r for (a, r) in zip(
        angles_sorted, clean_ranges_sorted) if abs(a) < BEAM_ANGLE]
    fwd_clearance = min(fwd_ranges)

    right_ranges = [r for (a, r) in zip(
        angles_sorted, clean_ranges_sorted) if (a > 30 and a < 100)]
    right_clearance = min(right_ranges)

    left_ranges = [r for (a, r) in zip(
        angles_sorted, clean_ranges_sorted) if (a < -30 and a > -100)]
    left_clearance = min(left_ranges)

    print(
        f"\nCLEARANCE FWD: {fwd_clearance:8.4}"
        f" LEFT: {left_clearance:8.4}"
        f" RIGHT: {right_clearance:8.4}\n")


def robot_coordinates(x, y, x_robot, y_robot):
    distance = math.sqrt((x-x_robot)**2 + (y - y_robot)**2)
    direction = math.atan2((y-y_robot), (x-x_robot))
    return distance, direction


def move_fwd(velocity_publisher, speed):
    """ Straight motion method """

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # get current location from the global variable before entering the loop
    global x, y

    # use clearances from the global variables
    # (constantly updated by scan_callback())
    global fwd_clearance, left_clearance, right_clearance

    global SAFETY_DIST, RATE

    # save initial coordinates
    x0 = x
    y0 = y

    velocity_message.linear.x = speed

    distance_moved = 0.0
    # we publish the velocity at RATE Hz (RATE times per second)
    loop_rate = rospy.Rate(RATE)

    rospy.loginfo("Straight motion")
    while True:

        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

        distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print(
            f"** Moving fwd: {distance_moved:6.4}m     Pose: {x:6.4}m, {y:6.4}m, {math.degrees(yaw):6.4}deg")
        if fwd_clearance < SAFETY_DIST:
            rospy.loginfo("** Obstacle reached")
            break

    rospy.loginfo("** Stopping")
    # stop the robot when obstacle is reached
    velocity_message.linear.x = 0.0
    velocity_publisher.publish(velocity_message)


def follow_wall(velocity_publisher, speed):
    """ Follow wall method """

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # get current location from the global variable before entering the loop
    global x, y, yawa

    # use clearances from the global variables
    # (constantly updated by scan_callback())
    global fwd_clearance, left_clearance, right_clearance


def rotate_in_place(velocity_publisher, omega_degrees):
    """ Rotation in place method """

    # use clearances from the global variables
    # (constantly updated by scan_callback())
    global fwd_clearance, left_clearance, right_clearance

    global RATE, MIN_CLEARANCE
    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    omega = math.radians(omega_degrees)

    # publish velocity message to rotate
    velocity_message.angular.z = omega
    # at RATE Hz (RATE times per second)
    loop_rate = rospy.Rate(RATE)

    # get initial timestamp
    t0 = rospy.Time.now().to_sec()

    while True:

        velocity_publisher.publish(velocity_message)

        # get initial timestamp
        t1 = rospy.Time.now().to_sec()
        curr_yaw_degrees = (t1-t0)*omega_degrees
        loop_rate.sleep()

        print(
            f"** Rotating: {curr_yaw_degrees:6.4}deg    Pose: ({x:6.4}m, {y:6.4}m, {math.degrees(yaw):6.4}deg)")
        if fwd_clearance > MIN_CLEARANCE:
            rospy.loginfo("** Found clearance")
            break

    # stop the robot after the angle is reached
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)
    rospy.loginfo("** Stopping rotation")


def bug0_robot(velocity_publisher, goal_x, goal_y):
    """ BUG0 """

    # use current location from the global variables
    # (constantly updated by odom_callback())
    global x, y, yaw

    global fwd_clearance

    # get global params
    global LASER_RANGE, BEAM_ANGLE
    global ANGLE_TOL, SAFETY_DIST, MIN_CLEARANCE
    global WAIT, LIN_SPEED, ROT_SPEED, RATE

    (goal_dist, goal_angle) = robot_coordinates(goal_x, goal_y, x, y)
    print(f"** Goal at {goal_dist}(m), {goal_angle*180/math.pi}(deg)\n\n")
    time.sleep(1)

    # rotate until pointing to goal
    while abs(yaw - goal_angle) > ANGLE_TOL:
        rotate_in_place(ROT_SPEED)
    print("** Pointing at goal\n\n")
    time.sleep(1)

    while True:
        # Behavior 1: move fwd until blocked by obstacle
        print("** BEHAVIOR 1: MOVING FORWARD\n\n")
        move_fwd(velocity_publisher=velocity_publisher,
                 speed=LIN_SPEED)
        print("** REACHED OBSTACLE\n\n")
        time.sleep(WAIT)

        # Behavior 2: follow wall until clearance found
        print("** BEHAVIOR 2: FOLLOW WALL\n\n")
        follow_wall(velocity_publisher=velocity_publisher,
                    speed=LIN_SPEED)
        print("** DIRECTION TO GOAL CLEAR FROM OBSTACLES\n\n")
        time.sleep(WAIT)


if __name__ == '__main__':
    try:

        # declare the node
        rospy.init_node('bumper_node', anonymous=True)

        # declare velocity publisher
        cmd_vel_topic = '/cmd_vel'
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
        global ANGLE_TOL, SAFETY_DIST, MIN_CLEARANCE
        global WAIT, LIN_SPEED, ROT_SPEED, RATE

        LASER_RANGE = rospy.get_param("LASER_RANGE", 5.0)  # m
        BEAM_ANGLE = rospy.get_param("BEAM_ANGLE", 5.0)  # degrees

        ANGLE_TOL = rospy.get_param("ANGLE_TOL", 1.0)  # degrees
        SAFETY_DIST = rospy.get_param("SAFETY_DIST", 0.6)  # m
        MIN_CLEARANCE = rospy.get_param("MIN_CLEARANCE", 3.0)  # m

        WAIT = rospy.get_param("WAIT", .5)  # s
        LIN_SPEED = rospy.get_param("LIN_SPEED", 0.4)  # m/s
        ROT_SPEED = rospy.get_param("ROT_SPEED", 15.0)  # degrees/s
        RATE = rospy.get_param("RATE", 10.0)  # Hz

        GOAL_X = rospy.get_param("GOAL_X", 1.0)  # m
        GOAL_Y = rospy.get_param("GOAL_X", 2.0)  # m

        # launch the bouncy robot app
        bug0_robot(velocity_publisher, GOAL_X, GOAL_Y)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
