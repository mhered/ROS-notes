#!/usr/bin/env python

from random import uniform
import rospy
from mhered.msg import IoTSensor

def publisher():
    # create a new publisher: 
    # specify topic name, type of message & queue size

    pub = rospy.Publisher('iot_sensor_topic', IoTSensor, queue_size=10)

    # initialize the node 
    # anonymous=True flag so that rospy will choose a unique name

    rospy.init_node('IoT_Publisher_node', anonymous=True)
    #set the loop rate
    rate = rospy.Rate(1) # 1Hz

    #keep publishing until a Ctrl-C is pressed
    sensor_msg = IoTSensor()
    sensor_msg.id = 1
    sensor_msg.name = "sensor #1"
    
    i = 0 
    while not rospy.is_shutdown():
        sensor_msg.temp = 25.4 + uniform(-1, 1)
        sensor_msg.humidity = 33 + uniform(-3, 3)
        
        rospy.loginfo(f"Publication #{i}\n {sensor_msg}")
        pub.publish(sensor_msg)
        rate.sleep()
        i+=1

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
