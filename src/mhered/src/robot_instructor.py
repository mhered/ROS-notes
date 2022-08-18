#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
vels = Twist()


def scan_callback(a):
    move(a)


def move(s):
    loop_rate = rospy.Rate(10)
    vels.linear.x = 0.4
    vels.angular.z = 0

    # stop moving if obstacles detected closer than 0.6 in ranges -5:5
    if(s.ranges[0] < 0.6 or s.ranges[1] < 0.6 or s.ranges[2] < 0.6 or s.ranges[3] < 0.6 or s.ranges[4] < 0.6 or s.ranges[5] < 0.6 or s.ranges[-5] < 0.6 or s.ranges[-4] < 0.6 or s.ranges[-3] < 0.6 or s.ranges[-2] < 0.6 or s.ranges[-1] < 0.6):
        vels.linear.x = 0
        # publish vels to /cmd_vel
        pub.publish(vels)

    # if stopped, rotate
    if(vels.linear.x == 0):
        rotate(s)
        # publish vels to /cmd_vel
        pub.publish(vels)

    # publish vels to /cmd_vel
    loop_rate.sleep()
    pub.publish(vels)


def rotate(b):
    loop_rate = rospy.Rate(10)
    # stop rotating if clearance >3 ahead
    if (b.ranges[0] < 3 or b.ranges[1] < 3 or b.ranges[2] < 3 or b.ranges[3] < 3 or b.ranges[4] < 3 or b.ranges[5] < 3):
        vels.angular.z = 1
        # pub.publish(vels)
    elif (b.ranges[-5] < 3 or b.ranges[-4] < 3 or b.ranges[-3] < 3 or b.ranges[-2] < 3 or b.ranges[-1] < 3):
        vels.angular.z = -1
        # pub.publish(vels)
    else:
        pub.publish(vels)

    pub.publish(vels)
    loop_rate.sleep()


if __name__ == "__main__":
    # launch node
    rospy.init_node('scan_node12', anonymous=True)
    # declare publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    # declare subscriber to /scan
    rospy.Subscriber("scan", LaserScan, scan_callback)
    time.sleep(2)
    rospy.spin()
