#!/usr/bin/env python


import time

import rospy

# import service definition
from ros_service_assignment.srv import (
    RectangleAreaService,
    RectangleAreaServiceRequest,
    RectangleAreaServiceResponse,
)


def handle_area_rect(request):
    area = request.width * request.height
    print(f"Server will return: {request.width} + {request.height} = {area}")
    time.sleep(2)  # 2 seconds
    return RectangleAreaServiceResponse(area)


def area_rect_server():

    rospy.init_node("area_rect_server")

    # create service listening to incoming requests
    # given service name, type and handle function
    my_service = rospy.Service("area_rect", RectangleAreaService, handle_area_rect)

    print("Server is ready to compute areas.")
    # start service to wait for incoming requests
    rospy.spin()


if __name__ == "__main__":
    area_rect_server()
