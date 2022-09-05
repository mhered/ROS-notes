#!/usr/bin/env python

import cv2
import rospy
from ball_detection import process_frame
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def image_callback(ros_image):
    print("received frame")
    global bridge, frame_wait_ms

    # convert ros_image into an opencv-compatible image frame
    try:
        rgb_frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print("Error in CVbridge: ", e)

    # 1. process frame
    processed_frame = process_frame(rgb_frame)

    # 2. display processed_frame
    cv2.imshow("Frame", processed_frame)

    # 3. wait
    cv2.waitKey(frame_wait_ms)


def main():
    rospy.init_node("image_converter", anonymous=True)

    # create a bridge
    global bridge
    bridge = CvBridge()

    global frame_wait_ms
    frame_wait_ms = 1

    # create a subscriber node and subscribe to a camera
    # for turtlebot3 waffle
    # image_topic="/camera/rgb/image_raw/compressed"
    # for usb cam
    # image_topic = "/usb_cam/image_raw"
    # for task2
    image_topic = "/tennis_ball_image"

    image_sub = rospy.Subscriber(image_topic, Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
