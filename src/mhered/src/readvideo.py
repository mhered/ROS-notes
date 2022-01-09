#!/usr/bin/env python

import numpy as np
import cv2

video_capture = cv2.VideoCapture(0)

while (True):
    ret, frame = video_capture.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
    cv2.rectangle(frame, (10, 10), (211, 211), (255, 0, 0), 5)
    cv2.imshow("Frame", frame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()
