#!/usr/bin/env python3

import time

import numpy as np


def wrap_to_180(angle):
    wrapped_angle = ((angle + 180) % 360) - 180
    return wrapped_angle


def find_nearest(array, value):
    """
    Given an ``array`` , and given a ``value`` , returns an index j such that
    ``value`` is between array[j] and array[j+1].
    ``array`` must be monotonic increasing.
    j=-1 or j=len(array) is returned to indicate that ``value`` is out of range
     below and above respectively.
    Source: https://stackoverflow.com/a/41856629/15472802
    """

    n = len(array)
    if value < array[0]:
        return -1
    elif value > array[n - 1]:
        return n
    jl = 0  # Initialize lower
    ju = n - 1  # and upper limits.
    while ju - jl > 1:  # If we are not yet done,
        jm = (ju + jl) >> 1  # compute a midpoint with a bitshift
        if value >= array[jm]:
            jl = jm  # and replace either the lower limit
        else:
            ju = jm  # or the upper limit, as appropriate.
        # Repeat until the test condition is satisfied.
    if value == array[0]:  # edge cases at bottom
        return 0
    elif value == array[n - 1]:  # and top
        return n - 1
    else:
        return jl


if __name__ == "__main__":
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

    """
    angles= np.asarray(range(0, 360, 30)
    wrapped_angles = np.asarray(range(-180, 180, 30))
    ids= wrapped_angles.argsort()
    wrapped_sorted = wrapped_angles[ids[::-1]]
    """
