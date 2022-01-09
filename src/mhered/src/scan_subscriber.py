#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
import datetime

"""
definition of sensor_msgs.msg/LaserScan:

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
"""


def scan_callback(message):
    timestamp = message.header.stamp
    secs = timestamp.secs
    nsecs = timestamp.nsecs
    time_in_secs = secs + (nsecs/1000000000)
    formatted_time = datetime.datetime.fromtimestamp(time_in_secs).isoformat()
    formatted_msg = f"Scan received on {formatted_time}"
    # get_caller_id(): Get fully resolved name of local node
    rospy.loginfo(rospy.get_caller_id() + "\n" + formatted_msg)

    global axes, line

    ydata = np.asarray(message.ranges)
    xdata = 180/np.pi*(message.angle_min +
                       message.angle_increment * np.asarray(range(len(ydata))))

    # update plot
    line.set_data(xdata, ydata)

    # adjust axes limits to dataset + margin (in percentage)
    margin = 0.05
    axes.set_xlim(min(xdata)*(1-margin), max(xdata)*(1+margin))
    # remove Inf and NaN values from ydata to compute axes limits
    clean_ydata = ydata
    clean_ydata[~np.isfinite(clean_ydata)] = 0
    axes.set_ylim(min(clean_ydata)*(1-margin), max(clean_ydata)*(1+margin))


def scan_subscriber():
    # init new node
    rospy.init_node('scan_subscriber_node', anonymous=True)
    # subscribe to topic /scan
    rospy.Subscriber("scan", LaserScan, scan_callback)

    # spin() keeps python from exiting until this node is stopped
    # rospy.spin()

    # replaced by this as seen here:
    # https://stackoverflow.com/questions/35145555/python-real-time-plotting-ros-data
    plt.show(block=True)


if __name__ == '__main__':

    # turn on interactive mode
    plt.ion()
    # create a figure and axes
    global axes, line
    fig, axes = plt.subplots()
    line, = axes.plot([], 'r-')
    plt.show()

    scan_subscriber()
