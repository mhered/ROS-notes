#!/usr/bin/env python3

import math
import time
from math import pi

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# topics
laserDataTopic = "/scan"
velCmdTopic = "/cmd_vel"
odomTopic = "/odom"

# simulatipon variables
half_front_vision_range = 15
LIN_VEL = 0.2
LIN_VEL_accel = 300
ANG_VEL = pi / 12
min_dist = 0.6
delimiting_sensor_range = 3
kpLinear = 0.15
kpAngular = 0.1


class cTurtle:
    def __init__(self) -> None:
        self.front_vision = []
        self.Vx = 0
        self.center_ray_index = -1
        self.process_time = 0.000043

        # configuration of velocity messages
        # moving message
        self.vel_msg = Twist()
        self.vel_msg.linear.x = LIN_VEL
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

        # stopping message
        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0
        self.stop_msg.linear.y = 0
        self.stop_msg.linear.z = 0
        self.stop_msg.angular.x = 0
        self.stop_msg.angular.y = 0
        self.stop_msg.angular.z = 0

        # left rotation message
        self.rot_left_msg = Twist()
        self.rot_left_msg.linear.x = 0
        self.rot_left_msg.linear.y = 0
        self.rot_left_msg.linear.z = 0
        self.rot_left_msg.angular.x = 0
        self.rot_left_msg.angular.y = 0
        self.rot_left_msg.angular.z = ANG_VEL

        # right rotation message
        self.rot_right_msg = Twist()
        self.rot_right_msg.linear.x = 0
        self.rot_right_msg.linear.y = 0
        self.rot_right_msg.linear.z = 0
        self.rot_right_msg.angular.x = 0
        self.rot_right_msg.angular.y = 0
        self.rot_right_msg.angular.z = -ANG_VEL

        rospy.init_node("somethingnotrequired", anonymous=True)
        self.rate = rospy.Rate(20)
        self.sub = rospy.Subscriber(laserDataTopic, LaserScan, self.scanCB)
        self.pub = rospy.Publisher(velCmdTopic, Twist, queue_size=0)
        self.od_sub = rospy.Subscriber(odomTopic, Odometry, self.odomCB)

    def odomCB(self, msg):
        x1 = msg.pose.pose.position.x
        y1 = msg.pose.pose.position.y
        # a1=self.a
        t0 = time.time()
        self.rate.sleep()
        x2 = msg.pose.pose.position.x
        y2 = msg.pose.pose.position.y
        # a2=self.a
        t1 = time.time()
        self.Vx == abs(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)) / (t1 - t0)
        # self.rate.sleep()

    def scanCB(self, message):
        self.front_vision = (
            message.ranges[-half_front_vision_range:]
            + message.ranges[:half_front_vision_range]
        )

    def move_straight(self):
        print(
            "moving straight\t[%.2f,\t%.2f,\t%.2f]"
            % (
                self.front_vision[0],
                self.front_vision[self.center_ray_index],
                self.front_vision[-1],
            )
        )
        if np.isinf(self.front_vision[self.center_ray_index]):
            self.vel_msg.linear.x = math.sqrt(3.5 - min_dist) * kpLinear + 0.05
        else:
            self.vel_msg.linear.x = (
                math.sqrt(
                    np.min(
                        [
                            self.front_vision[-1],
                            self.front_vision[self.center_ray_index],
                            self.front_vision[0],
                        ]
                    )
                    - min_dist
                )
                * kpLinear
                + 0.06
            )
        self.pub.publish(self.vel_msg)

    def stop_moving(self):
        print("stopping")
        self.pub.publish(self.stop_msg)

    def turn(self):
        # print(self.front_vision[0], self.front_vision[-1])
        if self.front_vision[-1] < self.front_vision[0]:
            while self.front_vision[-1] < delimiting_sensor_range:
                self.rot_right_msg.angular.z = -(
                    (delimiting_sensor_range - self.front_vision[-1]) * kpAngular + 0.05
                )
                print(
                    "turning right\t[%.2f,\t%.2f,\t%.2f]"
                    % (
                        self.front_vision[0],
                        self.front_vision[self.center_ray_index],
                        self.front_vision[-1],
                    )
                )
                self.pub.publish(self.rot_right_msg)
                self.rate.sleep()
        elif self.front_vision[-1] > self.front_vision[0]:
            while self.front_vision[0] < delimiting_sensor_range:
                self.rot_right_msg.angular.z = (
                    delimiting_sensor_range - self.front_vision[0]
                ) * kpAngular + 0.05
                print(
                    "turning left\t[%.2f,\t%.2f,\t%.2f]"
                    % (
                        self.front_vision[0],
                        self.front_vision[self.center_ray_index],
                        self.front_vision[-1],
                    )
                )
                self.pub.publish(self.rot_left_msg)
                self.rate.sleep()

    def core(self):
        self.start = time.process_time()
        if len(self.front_vision):
            self.center_ray_index = int(len(self.front_vision) / 2)
            if (
                self.front_vision[self.center_ray_index] <= min_dist
                or self.front_vision[-1] <= min_dist
                or self.front_vision[0] <= min_dist
            ):
                while self.Vx > 0.02:
                    self.stop_moving()
                    self.rate.sleep()
                self.turn()
            else:
                self.move_straight()
        else:
            self.stop_moving()
            print("NULL")
        self.rate.sleep()


if __name__ == "__main__":
    help_ = cTurtle()
    try:
        while 1:
            help_.core()
    except rospy.ROSInterruptException:
        for x in range(15):
            help_.stop_moving()
        print("\n\nNode Terminated\n")
    rospy.spin()
