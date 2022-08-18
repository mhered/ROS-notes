#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


distance = 99


def move(velocity_publisher):
    DIST = 0.6
    global distance

    velocity_message = Twist()

    loop_rate = rospy.Rate(10)
    while distance > DIST:

        linear_speed = 0.3
        velocity_message.linear.x = linear_speed
        velocity_publisher.publish(velocity_message)

    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


def rotate(velocity_publisher):
    MAX_DIST = 2
    global distance

    velocity_message = Twist()

    loop_rate = rospy.Rate(10)
    while distance < MAX_DIST:

        linear_speed = 0
        angular_speed = 1.5
        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)

    velocity_message.angular.x = 0
    velocity_publisher.publish(velocity_message)


def autonomous_robot(velocity_publisher):
    global distance

    velocity_message = Twist()

    loop_rate = rospy.Rate(10)
    while True:
        K_linear = 0.3
        K_angular = 0.5

        linear_speed = distance * K_linear
        angular_speed = 1/distance * K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)


def scan_callback(scan_data):
    global distance
    MIN_ANGLE = -10
    MAX_ANGLE = 10
    # Find minimum range

    distance = get_distance(scan_data.ranges, MIN_ANGLE, MAX_ANGLE)
    print('minumum distance: ', distance)


def get_distance(ranges, min_angle, max_angle):
    if min_angle < 0:
        range = ranges[0:max_angle] + ranges[min_angle:0]
    else:
        range = ranges[min_angle: max_angle]

    ranges = [x for x in range if not math.isnan(x)]

    return min(ranges)


if __name__ == '__main__':

    # init new a node and give it a name
    rospy.init_node('scan_node', anonymous=True)
    # subscribe to the topic /scan.
    rospy.Subscriber("scan", LaserScan, scan_callback)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # time.sleep(2)
    autonomous_robot(velocity_publisher)

    rospy.spin()
