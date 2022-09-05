#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def publisher():

    global video_capture
    global bridge

    # create a new publisher:
    # specify topic name, type of message & queue size
    # topic name is "tennis_ball_image"
    pub = rospy.Publisher("tennis_ball_image", Image, queue_size=10)

    # initialize the node
    # anonymous=True flag so that rospy will choose a unique name

    rospy.init_node("Video_Publisher_node", anonymous=True)
    # set the loop rate
    rate = rospy.Rate(50)  # 50Hz

    # keep publishing until a Ctrl-C is pressed
    sensor_msg = Image()

    i = 0
    while not rospy.is_shutdown():
        try:
            ret, rgb_frame = video_capture.read()
            sensor_msg = bridge.cv2_to_imgmsg(rgb_frame, "bgr8")
        except CvBridgeError as e:
            print(e)
        rospy.loginfo(f"Publication #{i}\n")
        pub.publish(sensor_msg)
        rate.sleep()
        i += 1


if __name__ == "__main__":

    global video_source, video_capture
    video_source = 0
    # video_source = 'tennis-ball-video.mp4'
    video_capture = cv2.VideoCapture(video_source)

    # create a bridge
    global bridge
    bridge = CvBridge()

    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

    video_capture.release()
    cv2.destroyAllWindows()
