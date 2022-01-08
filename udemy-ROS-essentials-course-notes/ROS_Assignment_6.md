#### **Task 1. Using OpenCV (Without ROS)**

In the first task, you will read a video file using just OpenCV  functionalities, (without using ROS). The video file is located on [GitHub repository in this link](https://github.com/aniskoubaa/ros_essentials_cpp/tree/master/src/topic03_perception/video) and is called [`tennis-ball-video.mp4`](https://github.com/aniskoubaa/ros_essentials_cpp/blob/master/src/topic03_perception/video/tennis-ball-video.mp4). So, the algorithm, will consists in the following steps:

- read the video file using OpenCV
- For each frame read
  - apply the ball detection algorithm to detect the ball(s) in the image
  - display the result of the ball detection
  - continue processing the video until a key is pressed.

To make your task easier, start from the code in the file [ball_detection.py](https://github.com/aniskoubaa/ros_essentials_cpp/blob/master/src/topic03_perception/ball_detection.py), which you use as a template and starting point. Make necessary  modification so that instead of reading a single image, you read a video stream and get the images from it, one by one. If you do not know how  to read a video file, refer to the lecture `OpenCV: Video Streams Input (Python)` in OpenCV section or to the code [read_video.py](https://github.com/aniskoubaa/ros_essentials_cpp/blob/master/src/topic03_perception/read_video.py). 

Here is a screenshot of a sample output

![img](https://img-c.udemycdn.com/redactor/raw/2018-08-09_17-01-49-4a6676d8788a8219ab6235696c6e0272.png)

#### **Task 2. Using OpenCV + ROS and Reading from a Video File**

In task 2, you will do the same thing as Task 1, but we will have two different files for the tracking application.

1. The first file is an Image publisher node file, which will read the video  stream frame by frame, and publishes them through a specific topic. 
2. The second file is an Image subscriber node file, which will subscribe to  the image topic of tennis ball, extracts the image from ROS topic,  converts it to an OpenCV image using `cv_bridge`, and applies the ball detection algorithm. Review the lecture about CvBridge: Bridging OpenCV and ROS or see the code [image_pub_sub.py](https://github.com/aniskoubaa/ros_essentials_cpp/blob/master/src/topic03_perception/image_pub_sub.py).

Here is the description of the two files. 

**Publisher ROS Node**

- Create a new node called `tennis_ball_publisher.py` (or cpp)
- You need to create a new ROS topic called `tennis_ball_image` of type `sensor_msgs/Image`. 
- After you read a frame from the video stream using OpenCV, you need to publish every frame on the topic `tennis_ball_image` 
- Loop the video forever and keep publishing the frames until the node is stopped. 

**Subscriber ROS Node**

- Create a new node called `tennis_ball_listener.py` (or cpp). Use the code the code [image_pub_sub.py](https://github.com/aniskoubaa/ros_essentials_cpp/blob/master/src/topic03_perception/image_pub_sub.py) as a reference. 
- You need to subscribe new ROS topic called `tennis_ball_image` of type `sensor_msgs/Image`. 
- In the image callback, you need to convert the receive frame into an  OpenCV format, then apply the ball_detection algorithm on the received  frame. Display every frame after detecting the ball, its contour and  surrounding circle. 

#### **Task 3. Using OpenCV + ROS and Reading from a USB Camera**

In task 3, you will do the same thing as Task 2 but the stream will be read from the USB camera. 

You will create only one file of a ROS node called `tennis_ball_usb_cam_tracker.py`. The node should do the following:

- Subscribe to the topic of video stream of type `sensor_msgs/Image` coming from the camera. How to find it? To find the topic name, you need first to start the usb_camera in ROS as follow.
  Open a terminal and write the command

  ```
  > roscore
  > rosrun usb_cam usb_cam_node _pixel_format:=yuyv
  ```

  in another terminal you can run `rostopic list` to see all the topics coming from the usb_cam. Choose the topic  containing a raw image and use it in your subscriber. Define a callback  function for this subscriber called `image_callback` where you will process every incoming frame. 

- In the image callback, you need to convert the receive frame into an  OpenCV format, then apply the ball_detection algorithm on the received  frame. Display every frame after detecting the ball, its contour and  surrounding circle.

- To test your code you need a  yellow ball. If you do not have any yellow ball, you can get any image  of a tennis ball on your mobile phone and then showing to your USB  camera and make it move to see how your application is able to track it. 

Congratulations, you are now able to process any image coming from a ROS node either from a USB camera or any other video  streaming sources. 

#### Questions for this assignment

**Task 1 Code: Using OpenCV (Without ROS)**

Attach a screenshot of the output of your work

**Task 2. Publisher Node**

**Note: please use {...} to format the instruction for better readability and make it easier for me to review.**

**Task 2. Subscriber Node**

Attach a screenshot of the output of your work

**Note: please use {...} to format the instruction for better readability and make it easier for me to review.**

**Task 3. Ball Tracking with USB Camera Stream**

Attach a screenshot of the output of your work

**Note: please use {...} to format the instruction for better readability and make it easier for me to review.**