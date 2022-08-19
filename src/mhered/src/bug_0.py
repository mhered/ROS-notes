#!/usr/bin/env python3

"""
** BUG0 method**
    Repeat:
        * go towards the goal until obstacle detected
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

    """
    print(
        f"\nCLEARANCE FWD: {fwd_clearance:8.4}"
        f" LEFT: {left_clearance:8.4}"
        f" RIGHT: {right_clearance:8.4}\n")
    """


def robot_coordinates(x, y, x_robot, y_robot):
    distance = math.sqrt((x-x_robot)**2 + (y - y_robot)**2)
    direction = math.atan2((y-y_robot), (x-x_robot))
    return distance, direction


def go_to(velocity_publisher, goal):
    """ Go to goal method """

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # use current location from the global variable
    # constantly updated by odom_callback()
    global x, y, yaw

    x_goal = goal[0]
    y_goal = goal[1]

    global THRESHOLD, K_DISTANCE, K_ANGLE
    global RATE, SAFETY_DIST

    # Publish the velocity at RATE Hz (RATE times per second)
    loop_rate = rospy.Rate(RATE)

    rospy.loginfo(f"Go to goal: {goal} from ({x}, {y})")

    while True:

        (distance_to_goal, angle_to_goal) = robot_coordinates(
            x_goal, y_goal, x, y)

        """
        rospy.loginfo(
            f"Goal at {distance_to_goal:6.3}m {math.degrees(angle_to_goal):6.3}deg Pose: ({x:6.3}m, {y:6.3}m, {yaw:6.3}rad)")
        """

        if distance_to_goal < THRESHOLD:
            rospy.loginfo("** GOAL REACHED\n\n")
            success = True
            break

        if fwd_clearance < SAFETY_DIST:
            rospy.loginfo("** OBSTACLE REACHED\n\n")
            success = False
            break

        vel_lin = min(0.5, K_DISTANCE * distance_to_goal)
        vel_ang = K_ANGLE * (angle_to_goal - yaw)
        rospy.loginfo(
            f"Goal at {angle_to_goal:6.3}rad yaw: {yaw:6.3}rad) vel_ang: {vel_ang:6.3}rad/s")

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


def follow_wall(velocity_publisher, speed):
    """ Follow wall method """

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # get current location from the global variable before entering the loop
    global x, y, yaw

    # use clearances from the global variables
    # (constantly updated by scan_callback())
    global fwd_clearance, left_clearance, right_clearance


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

    (dist_to_goal, yaw_to_goal) = robot_coordinates(goal_x, goal_y, x, y)
    print(f"** Goal at {dist_to_goal}(m), {math.degrees(yaw_to_goal)}(deg)\n\n")

    while True:
        # Behavior 1: go to goal unless blocked by obstacle
        rospy.loginfo("** BEHAVIOR 1: GO TO GOAL\n\n")
        time.sleep(WAIT)
        success = go_to(velocity_publisher=velocity_publisher,
                        goal=[goal_x, goal_y])
        if success:
            break
        else:
            rospy.loginfo("** REACHED OBSTACLE\n\n")
            time.sleep(WAIT)

        # Behavior 2: follow wall until clearance found
        rospy.loginfo("** BEHAVIOR 2: FOLLOW WALL\n\n")
        follow_wall(velocity_publisher=velocity_publisher,
                    speed=LIN_SPEED)
        rospy.loginfo("** DIRECTION TO GOAL CLEAR FROM OBSTACLES\n\n")
        time.sleep(WAIT)

    rospy.loginfo("** REACHED GOAL\n\n")
    time.sleep(WAIT)


if __name__ == '__main__':
    try:

        # declare the node
        rospy.init_node('bug0_node', anonymous=True)

        # declare velocity publisher
        cmd_vel_topic = '/cmd_vel'
        velocity_publisher = rospy.Publisher(
            cmd_vel_topic,
            Twist,
            queue_size=10)

        # declare /odom subscriber
        odom_topic = "/odom"
        odom_subscriber = rospy.Subscriber(
            odom_topic,
            Odometry,
            odom_callback)

        # declare /scan subscriber
        scan_topic = "/scan"
        scan_subscriber = rospy.Subscriber(
            scan_topic,
            LaserScan,
            scan_callback)

        # get ROS parameters (or default)
        global LASER_RANGE, BEAM_ANGLE
        global ANGLE_TOL, SAFETY_DIST, MIN_CLEARANCE
        global WAIT, RATE
        global LIN_SPEED, ROT_SPEED
        global THRESHOLD, K_DISTANCE, K_ANGLE

        LASER_RANGE = rospy.get_param("LASER_RANGE", 5.0)  # m
        BEAM_ANGLE = rospy.get_param("BEAM_ANGLE", 5.0)  # degrees

        ANGLE_TOL = rospy.get_param("ANGLE_TOL", 1.0)  # degrees
        SAFETY_DIST = rospy.get_param("SAFETY_DIST", 0.6)  # m
        MIN_CLEARANCE = rospy.get_param("MIN_CLEARANCE", 3.0)  # m

        WAIT = rospy.get_param("WAIT", .5)  # s
        LIN_SPEED = rospy.get_param("LIN_SPEED", 0.4)  # m/s
        # ROT_SPEED = rospy.get_param("ROT_SPEED", 15.0)  # degrees/s
        RATE = rospy.get_param("RATE", 10.0)  # Hz

        # for go_to()
        GOAL_X = rospy.get_param("GOAL_X", 1.0)  # m
        GOAL_Y = rospy.get_param("GOAL_X", -2.0)  # m

        THRESHOLD = rospy.get_param("THRESHOLD", 0.1)  # ?
        K_DISTANCE = rospy.get_param("K_DISTANCE", 0.25)  # ?
        K_ANGLE = rospy.get_param("K_ANGLE", 5.0)  # ?

        # launch the bug0 robot app
        time.sleep(5.0)
        rospy.loginfo("** LAUNCHING BUG0\n\n")
        bug0_robot(velocity_publisher=velocity_publisher,
                   goal_x=GOAL_X,
                   goal_y=GOAL_Y)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
