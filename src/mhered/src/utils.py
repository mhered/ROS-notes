#!/usr/bin/env python3
"""
Auxiliary functions for bug0, bug1, bug2 robot algorithms
get_t_from_stamp()
robot_coordinates()
get_clearance()
wrap_to_180()
find_nearest()
"""


import math
import time

import numpy as np  # pylint: disable=E0401


def get_t_from_stamp(stamp):
    """
    Receive a timestamp from a Header message
    (with fields stamp.secs and stamp.nsecs)
    Return t in seconds
    """

    sec = stamp.secs
    nsec = stamp.nsecs
    t = sec + nsec / 1e9  # 1s = 10**9 nanosecs
    return t


def robot_coordinates(x_goal, y_goal, x, y, yaw):
    """
    Return the relative distance and angle (in degrees wrapped at -180 180)
    of a goal (x_goal, y_goal) as seen from the robot at pose (x,y,yaw)
    """
    xR_goal = x_goal - x
    yR_goal = y_goal - y

    distance = math.sqrt(xR_goal**2 + yR_goal**2)
    direction_global = math.atan2(yR_goal, xR_goal)
    direction_relative = direction_global - yaw

    angle_to_goal_deg = wrap_to_180(math.degrees(direction_relative))

    return distance, angle_to_goal_deg


def get_clearance(angles, ranges, beam_dir, beam_aperture):
    """
    Take an array of ranges measured at specified angles
    Return clearance in a specified beam direction and aperture

    Note: angles, beam_dir, beam aperture must all be in consistent units
    (i.e. all in degrees or all in radians)
    """

    max_angle = beam_dir + beam_aperture
    min_angle = beam_dir - beam_aperture

    ranges_of_interest = [
        r for (a, r) in zip(angles, ranges) if min_angle < a < max_angle
    ]
    clearance = np.mean(ranges_of_interest)

    return clearance


def wrap_to_180(angle):
    """
    Wrap angle in degrees to [-180 : 180) range
    """
    wrapped_angle = ((angle + 180) % 360) - 180
    return wrapped_angle


def find_nearest(array, value):
    """
    Given an `array` , and given a `value` , returns an index j such that
    `value` is between array[j] and array[j+1].
    `array` must be monotonic increasing.
    j=-1 or j=len(array) is returned to indicate that `value` is out of range
     below and above respectively.
    Source: https://stackoverflow.com/a/41856629/15472802
    """

    n = len(array)

    # value out of range below
    if value < array[0]:
        return -1
    # and above
    if value > array[n - 1]:
        return n

    # edge cases at bottom
    if value == array[0]:
        return 0
    # and top
    if value == array[n - 1]:
        return n - 1

    # general case
    j_lower = 0  # Initialize lower
    j_upper = n - 1  # and upper limits.
    while j_upper - j_lower > 1:  # If we are not yet done,
        j_midpt = (j_upper + j_lower) >> 1  # compute midpoint with a bitshift
        if value >= array[j_midpt]:
            j_lower = j_midpt  # and replace either the lower limit
        else:
            j_upper = j_midpt  # or the upper limit, as appropriate.
        # Repeat until the test condition is satisfied.
    return j_lower


def main():
    """
    Testing (WIP)
    """
    ANGLE = -183.7

    angles = np.asarray(range(-180, 180, 1))
    ranges = angles / 10.0

    t0 = time.time()
    ind = find_nearest(angles, ANGLE)
    dt = time.time() - t0
    print(
        f"dt={dt*1e9:.1f}nsecs - "
        + f"find_nearest() - range at {angles[ind]:.2f}:"
        + f" {ranges[ind]:.2f}"
    )

    test_cases = [
        {
            "angle": 181.0,
            "wrap_result": -179.0,
        },
        {
            "angle": 0.0,
            "wrap_result": 0.0,
        },
        {
            "angle": 360.0,
            "wrap_result": 0.0,
        },
        {
            "angle": -181.0,
            "wrap_result": 179.0,
        },
        {
            "angle": -7.0 * 360.0 + 27.0,
            "wrap_result": 27.0,
        },
        {
            "angle": 11.0 * 360.0 - 53.0,
            "wrap_result": -53.0,
        },
        {
            "angle": np.asarray(range(0, 360, 30)),
            "wrap_result": np.asarray(
                [
                    0.0,
                    30.0,
                    60.0,
                    90.0,
                    120.0,
                    150.0,
                    -180.0,
                    -150.0,
                    -120.0,
                    -90.0,
                    -60.0,
                    -30.0,
                ]
            ),
        },
    ]

    for item in test_cases:
        wrapped_angle_i = wrap_to_180(item["angle"])
        print(f"angle = {item['angle']} --> wrapped = {repr(wrapped_angle_i)}")
        should_be_msg = f"should be {item['wrap_result']}"
        assert (
            np.round(wrapped_angle_i, 5) == item["wrap_result"]
        ).all(), should_be_msg

    print(wrap_to_180(np.asarray(range(0, 360, 30))))


if __name__ == "__main__":
    main()
