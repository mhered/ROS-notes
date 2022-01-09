#!/usr/bin/env python

import numpy as np

import cv2

image_name = "shapes.png"
title = image_name + " Image"
image_path = "/home/mhered/catkin_ws/src/mhered/src/images/"

# read image
img = cv2.imread(image_path+image_name)

# create a window
cv2.namedWindow(title, cv2.WINDOW_NORMAL)

# show image
cv2.imshow(title, img)

# wait for key press then close
cv2.waitKey(0)
cv2.destroyAllWindows()

# create a copy in copies folder (folder must exist)
copy_name = "copy-" + image_name
copy_full_path = image_path + "copies/" + copy_name

# print("Writing image ", copy_full_path)
cv2.imwrite(copy_full_path, img)

img_copy = cv2.imread(copy_full_path)

title_copy = copy_name + " Image"

# create a window
cv2.namedWindow(title_copy, cv2.WINDOW_NORMAL)

# show image
cv2.imshow(title_copy, img)

# wait for key press
cv2.waitKey(0)

# read image as BGR
img_color = cv2.imread(image_path+image_name, cv2.IMREAD_COLOR)
blue, green, red = cv2.split(img_color)
bgr_channels_img = np.concatenate((blue, green, red), axis=1)
cv2.imshow("BGR channels", bgr_channels_img)

# convert to HSV

img_hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(img_hsv)
hsv_channels_img = np.concatenate((h, s, v), axis=1)
cv2.imshow("HSV channels", hsv_channels_img)

# wait for key press then close
cv2.waitKey(0)
cv2.destroyAllWindows()
