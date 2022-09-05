#!/usr/bin/env python


import sys

import rospy

# import service definition
from mhered.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse


def add_two_ints_client(x, y):

    # keep client waiting until service is alive
    rospy.wait_for_service("add_two_ints")

    try:
        # create a client object, the service name must be same defined in the server
        my_request = rospy.ServiceProxy("add_two_ints", AddTwoInts)

        # what is this?
        response = my_request(x, y)
        return response.sum

    except rospy.ServiceException(e):
        print(f"Service call failed: {e}")


if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print("%s [x y]" % sys.argv[0])
        sys.exit(1)

    print(f"Client will request {x} + {y}...")
    result = add_two_ints_client(x, y)
    print(f"Server returned {x} + {y} = {result}")
