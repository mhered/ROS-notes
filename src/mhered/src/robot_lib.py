#!/usr/bin/env python3
"""
Library of robot reusable behaviours:
* rotate(vel_publisher, omega_degrees, angle_degrees, is_clockwise, rate=50)
* ...

"""
import math

import rospy
from geometry_msgs.msg import Twist


def rotate(vel_publisher, omega_degrees, angle_degrees, is_clockwise, rate=50):
    """Rotation in place method"""

    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    omega = math.radians(omega_degrees)

    if is_clockwise:
        velocity_message.angular.z = -abs(omega)
    else:
        velocity_message.angular.z = abs(omega)

    # publish the velocity at RATE Hz (RATE times per second)
    loop_rate = rospy.Rate(rate)

    rospy.loginfo("Rotation in place")

    # get initial timestamp
    t_0 = rospy.Time.now().to_sec()

    while True:

        vel_publisher.publish(velocity_message)

        # get initial timestamp
        t_1 = rospy.Time.now().to_sec()
        curr_yaw_degrees = (t_1 - t_0) * omega_degrees
        loop_rate.sleep()

        rospy.loginfo(
            f"Angle rotated: {curr_yaw_degrees:8.4}deg "
            # + f"Pose: ({x:8.4}m, {y:8.4}m, {math.degrees(yaw):8.4}deg)"
        )

        if curr_yaw_degrees >= angle_degrees:
            rospy.loginfo("Angle reached")
            break

    # stop the robot after the angle is reached
    velocity_message.angular.z = 0.0
    vel_publisher.publish(velocity_message)
