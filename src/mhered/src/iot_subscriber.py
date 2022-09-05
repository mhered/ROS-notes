#!/usr/bin/env python

import rospy

from mhered.msg import IoTSensor


def sensor_callback(message):

    id = message.id
    name = message.name
    temp = message.temp
    humidity = message.humidity

    formatted_msg = f"I heard: {id} {name} {temp:.5} {humidity:.5} "

    # get_caller_id(): Get fully resolved name of local node
    rospy.loginfo(rospy.get_caller_id() + "\n" + formatted_msg)


def subscriber():

    rospy.init_node("IoT_Subscriber_node", anonymous=True)

    rospy.Subscriber("iot_sensor_topic", IoTSensor, sensor_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    subscriber()
