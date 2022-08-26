#!/usr/bin/env python3
# Student 1: Tony Tao

import rospy
from yaml import scan
import roslib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from std_srvs.srv import Empty
import tf
import turtlesim.srv


def go_straight(xdist, ydist):
    angular = 1 * math.atan2(ydist, xdist)
    linear = 0.1 * math.sqrt(xdist ** 2 + ydist ** 2)
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    print(linear, angular)
    return cmd


def follow_wall(ranges, wall_thresh):
    cmd = Twist()
    wall_found = detect_walls2(ranges, wall_thresh)
    if wall_found:
        cmd.angular.z = -0.6
    else:
        cmd.linear.x = 0.2
    return cmd


def laserCallback(laserScan):
    global listener
    global tb3_velpub

    scans = laserScan.ranges
    wall_thresh = 0.3  # in meters
    goal_thresh = 0.05

    try:
        (trans, rot) = listener.lookupTransform('base_footprint', 'goal_frame', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("lookup failed")
    quaternion = rot
    rpy = tf.transformations.euler_from_quaternion(quaternion)
    print('translation vector: (', trans[0], ',', trans[1], ',', trans[2], ')')
    print('rotation angles: roll=', rpy[0], ' pitch=', rpy[1], ' yaw=', rpy[2])
    xdist = trans[0]
    ydist = trans[1]
    yaw = rpy[2]
    cmd = Twist()
    wall_detected = detect_walls(scans, xdist, ydist, yaw, wall_thresh)
    print(wall_detected)

    # case 1: reached goal
    if (math.sqrt(xdist**2 + ydist**2) < goal_thresh):
        tb3_velpub.publish(cmd)
        print("reached goal!!")

    # case 2: no obstacle detected
    elif not wall_detected:
        cmd = go_straight(xdist, ydist)
        tb3_velpub.publish(cmd)
        print("driving straight")

    # case 3: obstacle detected
    # follow the wall
    else:
        cmd = follow_wall(scans, wall_thresh)
        tb3_velpub.publish(cmd)
        print("following wall")
    print()


def get_minimum(lst):
    temp = []
    for i in lst:
        if (not math.isnan(i)):
            temp.append(i)
    return min(temp)


def detect_walls(scan_range, robotx, roboty, yaw, wall_thresh):
    if yaw < 0:
        yaw = math.pi + abs(yaw)
    yaw = -yaw
    range = 10
    length = len(scan_range)

    # angle to look (goal direction)
    lookangle = int(math.degrees(math.atan2(roboty, robotx) + yaw))

    # check look angle
    lookangleBroadCaster = tf.TransformBroadcaster()
    rot_quaternion = tf.transformations.quaternion_from_euler(0, 0, math.atan2(roboty, robotx))
    trans = (0, 0, 0)
    curr_time = rospy.Time.now()
    lookangleBroadCaster.sendTransform(
        trans, rot_quaternion, curr_time, "look_frame", "base_footprint")

    ##
    midIndex = lookangle
    if midIndex - range < 0:
        beam = scan_range[(midIndex - range) % length: length + 1] + \
            scan_range[0: midIndex + range + 1]
    elif midIndex + range >= length:
        beam = scan_range[0: midIndex + range - length] + scan_range[midIndex - range: length + 1]
    else:
        beam = scan_range[midIndex - range: midIndex + range + 1]
    min_dist = get_minimum(beam)
    print(min_dist)
    if min_dist < wall_thresh:
        return True
    return False


def detect_walls2(scan_range, wall_thresh):
    window = 50
    length = len(scan_range)
    beam = scan_range[length - window: length + 1] + scan_range[0: window]
    min_dist = get_minimum(beam)
    print(min_dist)
    if min_dist < wall_thresh:
        return True
    return False


def main():
    rospy.init_node('bug0')

    global listener
    global tb3_velpub

    listener = tf.TransformListener()
    listener.waitForTransform('map', 'goal_frame', rospy.Time(), rospy.Duration(10.0))

    tb3_velpub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("scan", LaserScan, laserCallback)
    rospy.spin()


if __name__ == "__main__":
    main()
