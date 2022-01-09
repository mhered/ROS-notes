#!/usr/bin/env python


import sys
import rospy

# import service definition
from ros_service_assignment.srv import RectangleAreaService, RectangleAreaServiceRequest, RectangleAreaServiceResponse


def area_rect_client(x, y):

    # keep client waiting until service is alive
    rospy.wait_for_service('area_rect')

    try:
        # create a client object, the service name must be same defined in the server
        my_request = rospy.ServiceProxy('area_rect', RectangleAreaService) 
   
        # what is this? 
        response = my_request(width, height)
        return response.area

    except rospy.ServiceException(e):
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    if len(sys.argv) == 3:
        width = int(sys.argv[1])
        height = int(sys.argv[2])
    else:
        print("%s [width heigth]"%sys.argv[0])
        sys.exit(1)
    
    print(f"Client will request {width} + {height}...")
    result = area_rect_client(width, height)
    print(f"Server returned {width} + {height} = {result}")
