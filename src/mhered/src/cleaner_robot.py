#!/usr/bin/env python3

import math
import time

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.msg import Pose

x = 0.0
y = 0.0
yaw = 0.0


def pose_callback(pose_message):
    """Pose callback method"""
    global x, y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta


def move(velocity_publisher, speed, distance, is_forward):
    """Straight motion method"""

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # get current location from the global variable before entering the loop
    global x, y
    # save initial coordinates
    x0 = x
    y0 = y

    if is_forward:
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(50)  # we publish the velocity at 10 Hz (10 times per second)

    rospy.loginfo("Straight motion")
    while True:

        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

        distance_moved = abs(math.sqrt(((x - x0) ** 2) + ((y - y0) ** 2)))
        print(
            f"Distance moved: {distance_moved:10.4} Pose: ({x:8.4}, {y:8.4}, {yaw:8.4})"
        )
        if not (distance_moved < distance):
            rospy.loginfo("Distance reached")
            break

    # stop the robot after the distance is reached
    velocity_message.linear.x = 0.0
    velocity_publisher.publish(velocity_message)


def rotate(velocity_publisher, omega_degrees, angle_degrees, is_clockwise):
    """Rotation in place method"""

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    omega = math.radians(omega_degrees)

    if is_clockwise:
        velocity_message.angular.z = -abs(omega)
    else:
        velocity_message.angular.z = abs(omega)

    loop_rate = rospy.Rate(50)  # we publish the velocity at 10 Hz (10 times per second)

    rospy.loginfo("Rotation in place")

    # get initial timestamp
    t0 = rospy.Time.now().to_sec()

    while True:

        velocity_publisher.publish(velocity_message)

        # get initial timestamp
        t1 = rospy.Time.now().to_sec()
        curr_yaw_degrees = (t1 - t0) * omega_degrees
        loop_rate.sleep()

        print(
            f"Angle rotated: {curr_yaw_degrees:10.4} Pose: ({x:8.4}, {y:8.4}, {yaw:8.4})"
        )
        if not (curr_yaw_degrees < angle_degrees):
            rospy.loginfo("Angle reached")
            break

    # stop the robot after the angle is reached
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)


def set_yaw(velocity_publisher, orientation_degrees):
    """Set absolute orientation method"""

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # get current location from the global variable before entering the loop
    global yaw

    yaw_degrees = math.degrees(yaw)

    # subtract angles, wrapping result to [-180, 180]
    angle_degrees = ((orientation_degrees - yaw_degrees + 180) % 360) - 180

    # rotate towards smallest angle difference
    if angle_degrees < 0:
        is_clockwise = True
    else:
        is_clockwise = False

    rotate(velocity_publisher, 15, abs(angle_degrees), is_clockwise)
    rospy.loginfo(f"Orientation set to {orientation_degrees}")


def go_to(velocity_publisher, goal):
    """Go to goal method"""

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # use current location from the global variable (constantly updated by pose_callback())
    global x, y, yaw

    x_goal = goal[0]
    y_goal = goal[1]

    THRESHOLD = 0.1
    K_DISTANCE = 0.6
    K_ANGLE = 15

    loop_rate = rospy.Rate(50)  # we publish the velocity at 10 Hz (10 times per second)

    rospy.loginfo(f"Go to goal: {goal}")

    while True:

        distance_to_goal = abs(math.sqrt(((x_goal - x) ** 2) + ((y_goal - y) ** 2)))
        angle_to_goal = math.atan2(y_goal - y, x_goal - x)

        print(
            f"Distance to goal: {distance_to_goal:10.4} Pose: ({x:8.4}, {y:8.4}, {yaw:8.4})"
        )

        if distance_to_goal < THRESHOLD:
            rospy.loginfo("Goal reached")
            break

        velocity_message.linear.x = K_DISTANCE * distance_to_goal
        velocity_message.angular.z = K_ANGLE * (angle_to_goal - yaw)

        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

    # stop the robot after the distance is reached
    velocity_message.linear.x = 0.0
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)


def spiral(velocity_publisher, omega, d_vel):
    """Spiral method"""

    # use current location from the global variable (constantly updated by pose_callback())
    global x, y

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    OMEGA = omega
    VEL_INCREMENT = d_vel

    loop_rate = rospy.Rate(50)  # we publish the velocity at 10 Hz (10 times per second)

    rospy.loginfo("Spiral")

    while x > 0.5 and x < 10.5 and y > 0.5 and y < 10.5:

        velocity_message.linear.x += VEL_INCREMENT
        velocity_message.angular.z = OMEGA

        print(
            f"Linear speed: {velocity_message.linear.x:10.4} Pose: ({x:8.4}, {y:8.4}, {yaw:8.4})"
        )

        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

    rospy.loginfo("Edge reached")

    # stop the robot after condition is fulfilled
    velocity_message.linear.x = 0.0
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)


def cleaner_app(velocity_publisher, WAIT, LIN_SPEED, ROT_SPEED):
    """Cleaner robot method"""

    # save initial position
    global x, y, yaw
    x0, y0, yaw0 = x, y, yaw

    go_to(velocity_publisher=velocity_publisher, goal=(0.5, 0.5))
    time.sleep(WAIT)

    set_yaw(velocity_publisher=velocity_publisher, orientation_degrees=90)
    time.sleep(WAIT)

    for i in range(6):

        move(
            velocity_publisher=velocity_publisher,
            speed=LIN_SPEED,
            distance=10.0,
            is_forward=True,
        )
        time.sleep(WAIT)

        rotate(
            velocity_publisher=velocity_publisher,
            omega_degrees=ROT_SPEED,
            angle_degrees=90,
            is_clockwise=True,
        )
        time.sleep(WAIT)

        move(
            velocity_publisher=velocity_publisher,
            speed=LIN_SPEED,
            distance=1.0,
            is_forward=True,
        )
        time.sleep(WAIT)

        rotate(
            velocity_publisher=velocity_publisher,
            omega_degrees=ROT_SPEED,
            angle_degrees=90,
            is_clockwise=True,
        )
        time.sleep(WAIT)

        move(
            velocity_publisher=velocity_publisher,
            speed=LIN_SPEED,
            distance=10.0,
            is_forward=True,
        )

        time.sleep(WAIT)

        rotate(
            velocity_publisher=velocity_publisher,
            omega_degrees=ROT_SPEED,
            angle_degrees=90,
            is_clockwise=False,
        )

        time.sleep(WAIT)

        move(
            velocity_publisher=velocity_publisher,
            speed=LIN_SPEED,
            distance=1.0,
            is_forward=True,
        )

        time.sleep(WAIT)

        rotate(
            velocity_publisher=velocity_publisher,
            omega_degrees=ROT_SPEED,
            angle_degrees=90,
            is_clockwise=False,
        )

    # return to initial position
    go_to(velocity_publisher=velocity_publisher, goal=(x0, y0))
    time.sleep(WAIT)

    set_yaw(
        velocity_publisher=velocity_publisher, orientation_degrees=math.degrees(yaw0)
    )


if __name__ == "__main__":
    try:
        rospy.init_node("my_turtle_pose_node", anonymous=True)

        # declare velocity publisher
        cmd_vel_topic = "/turtle1/cmd_vel"
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        # declare pose subscriber
        pose_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(pose_topic, Pose, pose_callback)

        time.sleep(1)

        # get ROS parameters (or default)
        WAIT = rospy.get_param("WAIT", 0.5)
        LIN_SPEED = rospy.get_param("LIN_SPEED", 4.0)
        ROT_SPEED = rospy.get_param("ROT_SPEED", 30.0)

        # call the function
        cleaner_app(velocity_publisher, WAIT, LIN_SPEED, ROT_SPEED)

        print("start reset: ")
        time.sleep(5)
        rospy.wait_for_service("reset")
        reset_turtle = rospy.ServiceProxy("reset", Empty)
        reset_turtle()
        print("end reset: ")

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
