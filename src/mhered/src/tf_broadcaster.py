#! /usr/bin/env python3

import rospy
import tf
from turtlesim.msg import Pose


def pose_callback(pose_msg, turtlename):
    """ """
    # create a transform broadcaster
    transform_broadcaster = tf.TransformBroadcaster()

    # obtain orientation quaternion from 2D Pose msg in degrees
    rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, pose_msg.theta)

    # assemble translation vector from 2D Pose msg
    translation_vector = [pose_msg.x, pose_msg.y, 0]

    # obtain time stamp
    time_stamp = rospy.Time.now()

    # publish transform between turtle frame and world
    # args: (translation, quaternion, time stamp, child frame, parent frame)
    transform_broadcaster.sendTransform(
        translation_vector,
        rotation_quaternion,
        time_stamp,
        turtlename + "_frame",
        "world",
    )


if __name__ == "__main__":

    # init code
    rospy.init_node("turtle_tf_broadcaster")

    # get param from launch file
    turtlename = rospy.get_param("~turtle")

    # subscribe to the Pose topic
    rospy.Subscriber("/%s/pose" % turtlename, Pose, pose_callback, turtlename)
    rospy.spin()
