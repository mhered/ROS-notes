#!/usr/bin/env python3

"""
** BUG0 method**
    Repeat:
        * go towards the goal until obstacle detected
        * follow obstacle boundary until no obstacle in direction of the goal
"""

import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_lib import rotate
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from utils import get_clearance, robot_coordinates, wrap_to_180

# use current location from the global variables
# (constantly updated by odom_callback())
global x, y, yaw
x = 0.0
y = 0.0
yaw = 0.0

# use clearances from the global variables
# (constantly updated by scan_callback())
global fwd_clearance, goal_clearance, right_clearance
fwd_clearance = 0.0
goal_clearance = 0.0
right_clearance = 0.0

# get global params
global LASER_RANGE, BEAM_ANGLE
global ANGLE_TOL, SAFETY_DIST, MIN_CLEARANCE
global WAIT, LIN_SPEED, ROT_SPEED, RATE


def odom_callback(msg):
    """
    Constantly extract robot pose from /odom nav_msgs/Odometry
    """

    # update global variables
    global x, y, yaw

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    q_as_list = [q.x, q.y, q.z, q.w]
    (_, _, yaw) = euler_from_quaternion(q_as_list)


def scan_callback(msg):
    """
    Constantly update global clearances from /scan sensor_msgs/LaserScan
    """

    # use clearances from the global variables
    # (constantly updated by scan_callback())
    global fwd_clearance, right_clearance, goal_clearance

    # parameters
    global BEAM_ANGLE, LASER_RANGE
    global GOAL_X, GOAL_Y

    ranges = np.asarray(msg.ranges)
    angles = np.degrees(
        msg.angle_min + msg.angle_increment * np.asarray(range(len(ranges)))
    )

    # wrap angles to [-180, 180)
    angles_wrapped = wrap_to_180(angles)

    angles_indeces = angles_wrapped.argsort()
    angles_sorted = angles_wrapped[angles_indeces[::-1]]
    ranges_sorted = ranges[angles_indeces[::-1]]

    # deep copy to avoid modifying ydata
    clean_ranges_sorted = ranges_sorted.copy()
    # remove Inf and NaN values from ranges
    clean_ranges_sorted[~np.isfinite(clean_ranges_sorted)] = LASER_RANGE

    # update clearances

    fwd_clearance = get_clearance(
        angles=angles_sorted,
        ranges=clean_ranges_sorted,
        beam_dir=0,
        beam_aperture=BEAM_ANGLE,
    )

    right_clearance = get_clearance(
        angles=angles_sorted,
        ranges=clean_ranges_sorted,
        beam_dir=-90,
        beam_aperture=BEAM_ANGLE,
    )

    (distance2goal, angle2goal_rads) = robot_coordinates(
        x_goal=GOAL_X, y_goal=GOAL_Y, x=x, y=y, yaw=yaw
    )

    angle2goal_deg = math.degrees(angle2goal_rads)
    if angle2goal_deg > 180:
        angle2goal_deg -= 360

    goal_clearance = get_clearance(
        angles=angles_sorted,
        ranges=clean_ranges_sorted,
        beam_dir=angle2goal_deg,
        beam_aperture=BEAM_ANGLE,
    )


def go_to(velocity_publisher, goal):
    """Go to goal method"""

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # use current location from the global variable
    # constantly updated by odom_callback()
    global x, y, yaw

    x_goal = goal[0]
    y_goal = goal[1]

    global THRESHOLD, K_DISTANCE, K_ANGLE
    global RATE, SAFETY_DIST
    global LIN_SPEED

    # Publish the velocity at RATE Hz (RATE times per second)
    loop_rate = rospy.Rate(RATE)

    rospy.loginfo(f"Go to goal: {goal} from ({x:10.4}, {y:10.4})")

    while True:

        (distance_to_goal, angle_to_goal) = robot_coordinates(
            x_goal=x_goal, y_goal=y_goal, x=x, y=y, yaw=yaw
        )

        if distance_to_goal < THRESHOLD:
            rospy.loginfo("** GOAL REACHED\n\n")
            success = True
            break

        if fwd_clearance < SAFETY_DIST:
            rospy.loginfo("** OBSTACLE REACHED\n\n")
            success = False
            break

        vel_lin = min(
            LIN_SPEED, K_DISTANCE * fwd_clearance, K_DISTANCE * distance_to_goal
        )

        vel_ang = K_ANGLE * angle_to_goal

        rospy.loginfo(
            # f"Pose: ({x:5.2}m, {y:5.2}m, {math.degrees(yaw):6.2}deg)" +
            f"Goal at {distance_to_goal:10.4}m "
            + f"{math.degrees(angle_to_goal):10.4}deg "
            + f"v: {vel_lin:10.4}m/s w: {vel_ang:10.4}rad/s"
        )

        velocity_message.linear.x = vel_lin
        velocity_message.angular.z = vel_ang

        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

    # stop the robot on exit
    rospy.loginfo("** Stopping go_to")
    velocity_message.linear.x = 0.0
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)
    return success


def follow_wall(velocity_publisher, goal):
    """Follow wall method"""

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # use current location from the global variable
    # constantly updated by odom_callback()
    global x, y, yaw

    x_goal = goal[0]
    y_goal = goal[1]

    # use clearances from the global variables
    # (constantly updated by scan_callback())
    global fwd_clearance, right_clearance, goal_clearance

    global THRESHOLD, K_DISTANCE, K_ANGLE
    global RATE, SAFETY_DIST, MIN_CLEARANCE
    global LIN_SPEED, ROT_SPEED

    # Publish the velocity at RATE Hz (RATE times per second)
    loop_rate = rospy.Rate(RATE)

    rospy.loginfo(f"Follow wall: {goal} from ({x:10.4}, {y:10.4})")

    # rotate 90deg anticlockwise
    rotate(
        velocity_publisher=velocity_publisher,
        omega_degrees=ROT_SPEED,
        angle_degrees=90,
        is_clockwise=False,
        rate=RATE,
    )

    # main loop to follow wall until clearance towards goal
    while True:

        (distance_to_goal, angle_to_goal) = robot_coordinates(
            x_goal=x_goal, y_goal=y_goal, x=x, y=y, yaw=yaw
        )

        vel_lin = min(LIN_SPEED, K_DISTANCE * fwd_clearance)
        vel_ang = K_ANGLE * (SAFETY_DIST - right_clearance)

        rospy.loginfo(
            # f"Pose: ({x:5.2}m, {y:5.2}m, {math.degrees(yaw):6.2}deg)" +
            f"Goal at {math.degrees(angle_to_goal):10.4}deg "
            + f"Clearances goal: {goal_clearance:10.4} "
            + f"right: {right_clearance:10.4} "
            + f"v: {vel_lin:10.4}m/s w: {vel_ang:10.4}rad/s"
        )

        if goal_clearance > min(distance_to_goal, MIN_CLEARANCE):
            rospy.loginfo("** CLEARANCE TO GOAL REACHED\n\n")
            break

        velocity_message.linear.x = vel_lin
        velocity_message.angular.z = vel_ang

        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

    # stop the robot on exit
    rospy.loginfo("** Stopping go_to")
    velocity_message.linear.x = 0.0
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)


def bug0_robot(velocity_publisher, goal_x, goal_y):
    """BUG0"""

    # use current location from the global variables
    # (constantly updated by odom_callback())
    global x, y, yaw

    global fwd_clearance

    # get global params
    global LASER_RANGE, BEAM_ANGLE
    global ANGLE_TOL, SAFETY_DIST, MIN_CLEARANCE
    global WAIT, LIN_SPEED, ROT_SPEED, RATE

    # initial message
    (dist_to_goal, angle_to_goal) = robot_coordinates(
        x_goal=goal_x, y_goal=goal_y, x=x, y=y, yaw=yaw
    )

    rospy.loginfo(
        f"** Goal at {dist_to_goal:10.4}(m), "
        + f"{math.degrees(angle_to_goal):10.4}(deg)\n\n"
    )

    # main loop
    while True:
        # Behavior 1: go to goal unless blocked by obstacle
        rospy.loginfo("** BEHAVIOR 1: GO TO GOAL\n\n")
        time.sleep(WAIT)
        success = go_to(velocity_publisher=velocity_publisher, goal=[goal_x, goal_y])
        if success:
            # goal is reached, so escape the loop
            break
        # otherwise an obstacle was found
        rospy.loginfo("** REACHED OBSTACLE\n\n")
        time.sleep(WAIT)

        # Behavior 2: follow wall until clearance found
        rospy.loginfo("** BEHAVIOR 2: FOLLOW WALL\n\n")
        follow_wall(velocity_publisher=velocity_publisher, goal=[goal_x, goal_y])

        rospy.loginfo("** DIRECTION TO GOAL CLEAR FROM OBSTACLES\n\n")
        time.sleep(WAIT)

    rospy.loginfo("** EXIT BUG0\n\n")
    time.sleep(WAIT)


if __name__ == "__main__":
    try:

        # declare this node
        # Note: this name is overriden by the name used by the launch file
        rospy.init_node("my_bug0_node")

        # declare velocity publisher
        cmd_vel_topic = "/cmd_vel"
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        # declare /odom subscriber
        odom_topic = "/odom"
        odom_subscriber = rospy.Subscriber(odom_topic, Odometry, odom_callback)

        # declare /scan subscriber
        scan_topic = "/scan"
        scan_subscriber = rospy.Subscriber(scan_topic, LaserScan, scan_callback)

        # get ROS parameters (or defaults)
        global LASER_RANGE, BEAM_ANGLE
        global ANGLE_TOL, SAFETY_DIST, MIN_CLEARANCE
        global WAIT, RATE
        global LIN_SPEED, ROT_SPEED
        global THRESHOLD, K_DISTANCE, K_ANGLE
        global GOAL_X, GOAL_Y

        LASER_RANGE = rospy.get_param("LASER_RANGE", 5.0)  # m
        BEAM_ANGLE = rospy.get_param("BEAM_ANGLE", 5.0)  # degrees

        ANGLE_TOL = rospy.get_param("ANGLE_TOL", 1.0)  # degrees
        SAFETY_DIST = rospy.get_param("SAFETY_DIST", 0.3)  # m
        MIN_CLEARANCE = rospy.get_param("MIN_CLEARANCE", 3.0)  # m

        WAIT = rospy.get_param("WAIT", 0.5)  # s
        LIN_SPEED = rospy.get_param("LIN_SPEED", 0.2)  # m/s
        ROT_SPEED = rospy.get_param("ROT_SPEED", 30.0)  # degrees/s
        RATE = rospy.get_param("RATE", 10.0)  # Hz

        GOAL_X = rospy.get_param("GOAL_X", 1.8)  # m
        GOAL_Y = rospy.get_param("GOAL_Y", -0.2)  # m

        THRESHOLD = rospy.get_param("THRESHOLD", 0.1)  # ?
        K_DISTANCE = rospy.get_param("K_DISTANCE", 0.25)  # ?
        K_ANGLE = rospy.get_param("K_ANGLE", 1.0)  # ?

        # wait 15secs before launching the bug0 robot app
        time.sleep(15.0)
        rospy.loginfo("** LAUNCHING BUG0\n\n")
        bug0_robot(velocity_publisher=velocity_publisher, goal_x=GOAL_X, goal_y=GOAL_Y)

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
