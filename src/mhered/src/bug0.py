#!/usr/bin/env python3

""" BUG0 stub methods """

"""
    BUG0
    Repeat:
        * move straight towards the goal until obstacle detected
        * follow obstacle boundary until no obstacle in direction of the goal
"""




import math
import time
def bug0():
    """ Unfinished BUG0 """
    # goal
    goal_x = 10
    goal_y = 10

    (goal_dist, goal_angle) = robot_coordinates(goal_x, goal_y, x, y)
    print(f"** Goal at {goal_dist}(m), {goal_angle*180/math.pi}(deg)\n\n")
    time.sleep(1)

    # rotate until pointing to goal
    while abs(angle - goal_angle) > ANGLE_TOL:
        rotate_in_place(OMEGA)
    print("** Pointing at goal\n\n")
    time.sleep(1)


global x, y, angle


def send_cmd_vel(vel, omega):
    global x, y, angle
    x += math.cos(angle) * vel * 0.1
    y += math.sin(angle) * vel * 0.1
    angle += omega * 0.1

    # wrap angle to +-180deg
    if angle > math.pi:
        angle -= 2*math.pi
    elif angle < -math.pi:
        angle += 2*math.pi

    print(f"x:{x} y:{y} a:{angle*180/math.pi}")
    time.sleep(.1)


def rotate_in_place(omega):
    send_cmd_vel(0, omega)


def move_fwd(vel):
    send_cmd_vel(vel, 0)


def stop():
    send_cmd_vel(0, 0)


def robot_coordinates(x, y, x_robot, y_robot):
    distance = math.sqrt((x-x_robot)**2 + (y - y_robot)**2)
    direction = math.atan2((y-y_robot), (x-x_robot))
    return distance, direction


def fwd_clearance():
    global x, y, angle, LASER_RANGE

    if angle < 0.1:
        clearance = LASER_RANGE
    else:
        clearance1 = (5-y) / math.sin(angle-5/180*math.pi)
        clearance2 = (5-y) / math.sin(angle+5/180*math.pi)
        clearance = min(clearance1, clearance2)
        if clearance > LASER_RANGE:
            clearance = LASER_RANGE
    print(f"fwd clearance:{clearance}")
    return clearance


def main():

    # robot position and angle
    global x, y, angle, LASER_RANGE
    x = 0
    y = 0
    angle = 0

    # params
    ANGLE_TOL = 0.02
    SAFETY_DIST = 0.3
    VEL = 3
    OMEGA = .4
    LASER_RANGE = 8

    # goal
    goal_x = 10
    goal_y = 10

    (goal_dist, goal_angle) = robot_coordinates(goal_x, goal_y, x, y)
    print(f"** Goal at {goal_dist}(m), {goal_angle*180/math.pi}(deg)\n\n")
    time.sleep(1)

    # rotate until pointing to goal
    while abs(angle - goal_angle) > ANGLE_TOL:
        rotate_in_place(OMEGA)
    print("** Pointing at goal\n\n")
    time.sleep(1)

    # move fwd until obstacle
    while fwd_clearance() > SAFETY_DIST:
        move_fwd(VEL)

    print("** Reached obstacle\n\n")
    time.sleep(1)

    # move fwd until y=5
    stop()

    print("** Stopped\n\n")
    time.sleep(1)

    # rotate until clearance >3
    while fwd_clearance() < 3:
        rotate_in_place(-OMEGA)

    print("** Found direction clear of obstacles\n\n")
    time.sleep(1)

    # move fwd a bit
    for i in range(10):
        move_fwd(VEL)


if __name__ == "__main__":
    main()
