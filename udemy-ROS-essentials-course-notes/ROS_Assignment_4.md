  In this exercise, you will develop a new ROS service by applying the concepts that you have learned in this section. The objective is to create a ROS service where a client sends two float  numbers, width and height to the server, then the server will respond  the area of the rectangle. First, create a new ROS package, and call it `ros_service_assignment`.You first need to define a service file called `RectangleAeraService.srv`.The request part should contains two float values: the width and the  height. Use float32 as a type. Refer to this ROS page for more  information about available types in ROS. http://wiki.ros.org/msg The response part should contain the area of the rectangle, which is width*height. Write a Client and Server that implements this application using Python or C++.Test your application and make sure that it works. 

Questions for this assignment

**Note: please use {...} to format the instruction for better readability and make it easier for me to review.**

1. What is the command used to create a ROS package called ros_service_assignment?Make sure to clarify the path where to create it. 

   from `~/catkin_ws/src/` issue:

   `$ catkin_create_pkg ros_service_assignment std_msgs rospy roscpp`

2. What is the name of the folder when to create the service file?Provide the absolute path to the file (from the root). 

   `/home/mhered/catkin_ws/src/ros_service_assignment/srv`

3. What is the content of the service file RectangleAreaService.srv?

   ```
   # Service definition file
   # request message
   float64 height
   float64 width
   ---
   # response message
   float64 area
   ```

4. What are the changes you need to do in the CMakeLists.txt. Copy/paste the whole CMakeLists.txt. 

   add dependencies:

   ```
   ```

   

5. What are the changes you need to do the package.xml? Copy/paste the whole package.xml.

   add dependencies to `message_generation` and `message_runtime`:

   ``` 
   ```

   

6. What is the command to build the new service and generate executable files?

   from `~/catkin_ws`issue:

   $ catkin_make

7. How to make sure that service files are now created?

   ```
   $ rossrv show ros_service_assignment/RectangleAreaService
   
   float32 height
   float32 width
   ---
   float32 area
   ```

8. **Note: please use {...} to format the instruction for better readability and make it easier for me to review.** Write the server application (C++ or Python)

   ```python
   #!/usr/bin/env python
   
   import rospy
   import time
   
   # import service definition
   from ros_service_assignment.srv import RectangleAreaService, RectangleAreaServiceRequest, RectangleAreaServiceResponse
   
   def handle_area_rect(request):
       area = request.width * request.height
       print(f"Server will return: {request.width} + {request.height} = {area}")
       time.sleep(2) # 2 seconds
       return RectangleAreaServiceResponse(area)
   
   def area_rect_server():
   
       rospy.init_node('area_rect_server')
       
       # create service listening to incoming requests
       # given service name, type and handle function
       my_service = rospy.Service('area_rect', RectangleAreaService, handle_area_rect)
       
       print('Server is ready to compute areas.')
       # start service to wait for incoming requests 
       rospy.spin()
   
   if __name__ == "__main__":
       area_rect_server()
   ```

   

9. **Note: please use {...} to format the instruction for better readability and make it easier for me to review. **Write the client application (C++ or Python)

   ```python
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
   
   ```

   

10. **Note: please use {...} to format the instruction for better readability and make it easier for me to review.**What are the commands to test that the application works fine.

Terminal 1:

`$ roscore`

Terminal 2:

```bash
$ rosrun ros_service_assignment area_rect_server.py
Server is ready to compute areas.
```

Terminal 3:

```bash
$ rosrun ros_service_assignment area_rect_client.py 2 3
Client will request 2 + 3...
Server returned 2 + 3 = 6.0
```

