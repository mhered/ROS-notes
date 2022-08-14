#!/usr/bin/env python3

import geometry_msgs.msg
import math
import rospy
import tf
import turtlesim.msg
import turtlesim.srv


if __name__ == "__main__":

    # init node
    rospy.init_node("turtle_tf_listener")

    # create a new transform listener
    transform_listener = tf.TransformListener()

    # spawn the follower turtlesim
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle_follower')

    turtle_follower_vel = rospy.Publisher(
        'turtle_follower/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (translation, rotation) = transform_listener.lookupTransform(
                '/turtle_follower_frame', '/turtle1_frame', rospy.Time(0))
        except (tf.LookupException,
                tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # relative coordinates of follower in turtle1_frame
        x = translation[0]
        y = translation[1]

        angular_vel = 4 * math.atan2(y, x)
        linear_vel = 0.5 * math.sqrt(x * x + y * y)

        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel

        turtle_follower_vel.publish(cmd)

        rate.sleep()
