#!/usr/bin/env python3

import roslib
import rospy
import math
import tf
import turtlesim.msg
from turtlesim.msg import Pose
import geometry_msgs.msg
import turtlesim.srv


if __name__ == "__main__":

    # init node
    rospy.init("turtle_tf_listener")

    # create a new transform listener
    transform_listener = tf.TransformListener()

    # spawn the follower turtlesim
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle_follower')

    turtle_follower_vel = rospy.Publisher('turtle_follower/cmd_vel',
                                          geometry_msgs.msg.Twist.queue_size=1)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (translation, rotation) = transform_listener.lookupTransform(
                '/turtle_follower_frame', '/turtle_leader_frame', rospy.Time(0))
        except (tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # relative coordinates of follower in turtle_leader_frame
        x = translation(0)
        y = translation(1)

        angular_vel = 4 * math.atan2(y, x)
        linear_vel = 0.5 * math.sqrt(x * x + y * y)

        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel

        turtle_follower_velocity.publish(cmd)

        rate.sleep()
