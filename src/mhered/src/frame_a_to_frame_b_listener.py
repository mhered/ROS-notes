#!/usr/bin/env python3

import rospy
import tf

if __name__ == "__main__":
    rospy.init_node("frame_a_frame_b_listener_node")

    listener = tf.TransformListener()
    rate = rospy.Rate(1.0)
    listener.waitForTransform("/frame_a", "/frame_b", rospy.Time(), rospy.Duration(4.0))

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                "/frame_a", "/frame_b", rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

        quaternion = rot
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        print("transformation between frame_a and frame_b detected")
        print(f"translation vector XYZ: ({trans[0]:.5}, {trans[1]:.5}, {trans[2]:.5})")
        print(f"rotation angles RPY: ({rpy[0]:.5}, {rpy[1]:.5}, {rpy[2]:.5})")

        rate.sleep()
