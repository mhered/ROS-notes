#!/usr/bin/env python3

"""
** BUG1 method**
    Repeat:
        * go towards the goal until obstacle detected
        * follow wall for a complete turn around the obstacle
            * record minimum distance to goal
        * follow wall point closest to goal
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
global x, y, yaw, t
x = 0.0
y = 0.0
yaw = 0.0
t = 0.0

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


def get_t_from_stamp(stamp):
    sec = stamp.secs
    nsec = stamp.nsecs
    t = sec + nsec / 1e9  # 1s = 10**9 nanosecs
    return t


def odom_callback(odom_message):
    """
    Constantly extract robot pose from /odom nav_msgs/Odometry
    """

    # update global variables
    global x, y, yaw, t

    x = odom_message.pose.pose.position.x
    y = odom_message.pose.pose.position.y

    q = odom_message.pose.pose.orientation
    q_as_list = [q.x, q.y, q.z, q.w]
    (_, _, yaw) = euler_from_quaternion(q_as_list)

    t = get_t_from_stamp(odom_message.header.stamp)


def get_clearance(angles, ranges, beam_dir, beam_aperture):
    """
    Takes an array of ranges measured at specified angles
    Return clearance in a specified beam direction and aperture

    Note: angles, beam_dir, beam aperture must all be in consistent units
    (i.e. all in degrees or all in radians)
    """
    ranges = [r for (a, r) in zip(
        angles, ranges) if (
        a > (beam_dir - beam_aperture) and a < (beam_dir + beam_aperture))]
    clearance = np.mean(ranges)
    return clearance


def scan_callback(message):
    """
    Constantly update global clearances from /scan sensor_msgs/LaserScan
    """

    # use clearances from the global variables
    # (constantly updated by scan_callback())
    global fwd_clearance, right_clearance, goal_clearance

    # parameters
    global BEAM_ANGLE, LASER_RANGE
    global GOAL_X, GOAL_Y

    ranges = np.asarray(message.ranges)
    angles = np.degrees((message.angle_min +
                         message.angle_increment * np.asarray(
                             range(len(ranges)))))

    # shift angles >180 from [180 - 360) to [-180, 0)
    angles_shifted = np.where(angles > 180, angles-360, angles)

    angles_indeces = angles_shifted.argsort()
    angles_sorted = angles_shifted[angles_indeces[::-1]]
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
        beam_aperture=BEAM_ANGLE)

    right_clearance = get_clearance(
        angles=angles_sorted,
        ranges=clean_ranges_sorted,
        beam_dir=-75,
        beam_aperture=BEAM_ANGLE)

    (distance2goal, angle2goal_rads) = robot_coordinates(
        GOAL_X, GOAL_Y, x, y, yaw)

    angle2goal_deg = math.degrees(angle2goal_rads)
    if angle2goal_deg > 180:
        angle2goal_deg -= 360

    goal_clearance = get_clearance(
        angles=angles_sorted,
        ranges=clean_ranges_sorted,
        beam_dir=angle2goal_deg,
        beam_aperture=BEAM_ANGLE)

    """
    print(
        f"\nCLEARANCE FWD: {fwd_clearance:8.4}" +
        f" GOAL: {goal_clearance:8.4}" +
        f" angle2goal: {angle2goal_deg:8.4}" +
        f" RIGHT: {right_clearance:8.4}\n"
    )
    """


def robot_coordinates(x_goal, y_goal, x, y, yaw):
    """ Returns distance and angle (rad) of goal relative to robot frame"""
    xR_goal = x_goal - x
    yR_goal = y_goal - y

    distance = math.sqrt(xR_goal**2 + yR_goal**2)
    direction_global = math.atan2(yR_goal, xR_goal)
    direction_relative = direction_global - yaw
    return distance, direction_relative


def rotate(velocity_publisher, omega_degrees, angle_degrees, is_clockwise):
    """ Rotation in place method """

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    omega = math.radians(omega_degrees)

    if (is_clockwise):
        velocity_message.angular.z = -abs(omega)
    else:
        velocity_message.angular.z = abs(omega)

    # publish the velocity at RATE Hz (RATE times per second)
    loop_rate = rospy.Rate(RATE)

    rospy.loginfo("Rotation in place")

    # get initial timestamp
    t0 = rospy.Time.now().to_sec()

    while True:

        velocity_publisher.publish(velocity_message)

        # get initial timestamp
        t1 = rospy.Time.now().to_sec()
        curr_yaw_degrees = (t1-t0)*omega_degrees
        loop_rate.sleep()

        rospy.loginfo(
            f"Angle rotated: {curr_yaw_degrees:8.4}deg Pose: ({x:8.4}m, {y:8.4}m, {math.degrees(yaw):8.4}deg)")
        if not(curr_yaw_degrees < angle_degrees):
            rospy.loginfo("Angle reached")
            break

    # stop the robot after the angle is reached
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)


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
    global LIN_SPEED

    # Publish the velocity at RATE Hz (RATE times per second)
    loop_rate = rospy.Rate(RATE)

    rospy.loginfo(f"Go to goal: {goal} from ({x:8.4}, {y:8.4})")

    while True:

        (distance_to_goal, angle_to_goal) = robot_coordinates(
            x_goal, y_goal, x, y, yaw)

        if distance_to_goal < THRESHOLD:
            rospy.loginfo("** GOAL REACHED\n\n")
            success = True
            break

        if fwd_clearance < SAFETY_DIST:
            rospy.loginfo("** OBSTACLE REACHED\n\n")
            success = False
            break

        vel_lin = min(
            LIN_SPEED,
            K_DISTANCE * fwd_clearance,
            K_DISTANCE * distance_to_goal)
        vel_ang = K_ANGLE * angle_to_goal

        rospy.loginfo(
            # f"Pose: ({x:5.2}m, {y:5.2}m, {math.degrees(yaw):6.2}deg)" +
            f"Goal at {distance_to_goal:8.4}m " +
            f"{math.degrees(angle_to_goal):8.4}deg " +
            f"v: {vel_lin:8.4}m/s w: {vel_ang:8.4}rad/s")

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


def turn_around_obstacle(velocity_publisher, goal, exit):
    """ Turn around obstacle method """

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # use current location from the global variable
    # constantly updated by odom_callback()
    global x, y, yaw, t

    x_goal = goal[0]
    y_goal = goal[1]

    x_exit = exit[0]
    y_exit = exit[1]

    # initialize search for point closest to goal
    x_closest = x_exit
    y_closest = y_exit
    (d_closest, _) = robot_coordinates(x_goal, y_goal, x_closest, y_closest, yaw)

    # use clearances from the global variables
    # (constantly updated by scan_callback())
    global fwd_clearance, right_clearance, goal_clearance

    global THRESHOLD, K_DISTANCE, K_ANGLE
    global RATE, SAFETY_DIST, MIN_CLEARANCE
    global LIN_SPEED, ROT_SPEED

    # Publish the velocity at RATE Hz (RATE times per second)
    loop_rate = rospy.Rate(RATE)

    rospy.loginfo(f"Follow wall until ({x_exit:8.4}, {y_exit:8.4}) " +
                  f"from ({x:8.4}, {y:8.4}) " +
                  f"with goal at ({x_goal:8.4}, {y_goal:8.4}) ")

    # rotate 90deg anticlockwise
    rotate(
        velocity_publisher=velocity_publisher,
        omega_degrees=ROT_SPEED,
        angle_degrees=90,
        is_clockwise=False)

    # initialize main loop
    iteration = 0
    last_t = t
    last_right_clearance = right_clearance
    time.sleep(WAIT)

    i = 0

    # main loop to follow wall until exit reached
    while True:
        iteration += 1

        # calculate distances to goal and exit points
        (distance_to_goal, angle_to_goal) = robot_coordinates(
            x_goal, y_goal, x, y, yaw)

        (distance_to_exit, _) = robot_coordinates(
            x_exit, y_exit, x, y, yaw)

        # if goal reached then exit
        if distance_to_goal < THRESHOLD:
            rospy.loginfo("** GOAL REACHED\n\n")
            break

        # if after some iterations exit point is reached then exit
        # need to allow some iterations otherwise the turn aborts
        if iteration > 15 and distance_to_exit < THRESHOLD:
            rospy.loginfo("** EXIT POINT REACHED\n\n")
            break

        # if current location is closest to goal then update it
        if distance_to_goal < d_closest:
            x_closest = x
            y_closest = y
            d_closest = distance_to_goal

        vel_lin = min(
            LIN_SPEED,
            K_DISTANCE * fwd_clearance)

        dt = t-last_t
        delta_clearance = right_clearance-last_right_clearance

        # PID for vel_ang

        Kp = - K_ANGLE
        e = right_clearance - SAFETY_DIST
        Ki = - 0.00
        i += e * dt
        Kd = - 0.1
        d = delta_clearance / dt

        vel_ang = Kp * e + Ki * i + Kd * d

        """
        # clip abs(vel_ang) to ROT_SPEED
        if vel_ang > 0:
            vel_ang = min(
                ROT_SPEED,
                vel_ang)
        elif vel_ang < 0:
            vel_ang = max(
                -ROT_SPEED,
                vel_ang)
        """

        velocity_message.linear.x = vel_lin
        velocity_message.angular.z = vel_ang

        velocity_publisher.publish(velocity_message)

        rospy.loginfo(
            # f"Pose: ({x:5.2}m, {y:5.2}m, {math.degrees(yaw):6.2}deg)" +
            f"Goal at {math.degrees(angle_to_goal):8.4}deg " +
            f"Clearances goal: {goal_clearance:8.4} right: {right_clearance:8.4} " +
            f"v: {vel_lin:8.4}m/s w: {vel_ang:8.4}rad/s")

        loop_rate.sleep()

    # stop the robot on exit
    rospy.loginfo("** Stopping follow_wall")
    velocity_message.linear.x = 0.0
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)
    return x_closest, y_closest


def bug1_robot(velocity_publisher, goal_x, goal_y):
    """ BUG1 """

    # use current location from the global variables
    # (constantly updated by odom_callback())
    global x, y, yaw

    global fwd_clearance

    # get global params
    global LASER_RANGE, BEAM_ANGLE
    global ANGLE_TOL, SAFETY_DIST, MIN_CLEARANCE
    global WAIT, LIN_SPEED, ROT_SPEED, RATE

    (dist_to_goal, angle_to_goal) = robot_coordinates(goal_x, goal_y, x, y, yaw)
    rospy.loginfo(
        f"** Goal at {dist_to_goal:8.4}(m), {math.degrees(angle_to_goal):8.4}(deg)\n\n")

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
            # remember start of turn around obstacle
            x0 = x
            y0 = y
            time.sleep(WAIT)

        # Behavior 2: follow wall for a full turn around obstacle
        rospy.loginfo("** BEHAVIOR 2: FOLLOW WALL\n\n")
        x_closest, y_closest = turn_around_obstacle(
            velocity_publisher=velocity_publisher,
            goal=[goal_x, goal_y],
            exit=[x0, y0])
        rospy.loginfo("** FULL TURN AROUND OBSTACLE COMPLETED\n\n")
        time.sleep(WAIT)

        # Behavior 3: follow wall until closest_pt
        rospy.loginfo("** BEHAVIOR 2: FOLLOW WALL\n\n")
        _ = turn_around_obstacle(
            velocity_publisher=velocity_publisher,
            goal=[goal_x, goal_y],
            exit=[x_closest, y_closest])
        rospy.loginfo("** CLOSEST PT TO GOAL REACHED\n\n")
        time.sleep(WAIT)

    rospy.loginfo("** EXIT BUG1\n\n")
    time.sleep(WAIT)


if __name__ == '__main__':
    try:

        # declare the node
        # Note: this name is overriden by the name used by the launch file
        rospy.init_node('my_bug1_node')

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
        global GOAL_X, GOAL_Y

        LASER_RANGE = rospy.get_param("LASER_RANGE", 5.0)  # m
        BEAM_ANGLE = rospy.get_param("BEAM_ANGLE", 5.0)  # degrees

        ANGLE_TOL = rospy.get_param("ANGLE_TOL", 1.0)  # degrees
        SAFETY_DIST = rospy.get_param("SAFETY_DIST", 0.5)  # m
        MIN_CLEARANCE = rospy.get_param("MIN_CLEARANCE", 3.0)  # m

        WAIT = rospy.get_param("WAIT", .5)  # s
        LIN_SPEED = rospy.get_param("LIN_SPEED", 0.2)  # m/s
        ROT_SPEED = rospy.get_param("ROT_SPEED", 30.0)  # degrees/s
        RATE = rospy.get_param("RATE", 10.0)  # Hz

        # for go_to()
        # (1,-1.8) to test only go_to
        # (2, -0.8) to test follow_wall
        GOAL_X = rospy.get_param("GOAL_X", 1.8)  # m
        GOAL_Y = rospy.get_param("GOAL_Y", -0.2)  # m

        THRESHOLD = rospy.get_param("THRESHOLD", 0.1)  # ?
        K_DISTANCE = rospy.get_param("K_DISTANCE", 0.2)  # ?
        K_ANGLE = rospy.get_param("K_ANGLE", .8)  # ?

        # launch the bug1 robot app
        time.sleep(15.0)
        rospy.loginfo("** LAUNCHING BUG1\n\n")
        bug1_robot(velocity_publisher=velocity_publisher,
                   goal_x=GOAL_X,
                   goal_y=GOAL_Y)

        # Do I need this? How to use rospy.spin()?
        # If I remove it the node dies after reaching the goal
        # If I keep it the node stays alive but does nothing
        # rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
