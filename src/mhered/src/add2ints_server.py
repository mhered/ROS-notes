#!/usr/bin/env python


import rospy
# import time


# import service definition
from mhered.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse

def handle_add_two_ints(request):
    sum = request.a + request.b
    print(f"Server will return: {request.a} + {request.b} = {sum}")
    # time.sleep(2) # 2 seconds
    return AddTwoIntsResponse(sum)

def add_two_ints_server():

    rospy.init_node('add_two_ints_server')
    
    # create service listening to incoming requests
    # given service name, type and handle function
    my_service = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    
    print('Server is ready to add integers.')
    # start service to wait for incoming requests 
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()