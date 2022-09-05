#!/usr/bin/env python

import cv2
import numpy as np

# global variables
global color_lower_bound, color_upper_bound
global is_bkg_black

is_bkg_black = False

# tennis ball in video
color_lower_bound = (30, 150, 100)
color_upper_bound = (50, 255, 255)

# tennis ball in my camera
# color_lower_bound = (20, 90, 150)
# color_upper_bound = (30, 150, 255)

# Kinga bottle cap
# color_lower_bound = (100, 80, 80)
# color_upper_bound = (110, 255, 255)

# Oranges
# color_lower_bound = (0, 120, 120)
# color_upper_bound = (23, 255, 255)


def filter_color(rgb_image, color_lower_bound, color_upper_bound):
    # convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

    # define a mask using the lower and upper bounds of the yellow color
    mask = cv2.inRange(hsv_image, color_lower_bound, color_upper_bound)

    return mask


def find_contours(mask):
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    return contours


def draw_ball_contours(rgb_image, contours, is_bkg_black):
    if is_bkg_black:
        contours_image = np.zeros(rgb_image.shape, "uint8")
    else:
        contours_image = rgb_image

    for c in contours:
        area = cv2.contourArea(c)

        if area > 100:
            # add contour in green
            cv2.drawContours(contours_image, [c], -1, (150, 250, 150), 2)

            # add contour centroid pink
            cx, cy = get_contour_centroid(c)
            cv2.circle(contours_image, (cx, cy), 5, (150, 150, 255), -1)

            # add min enclosing circle in yellow
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            cv2.circle(contours_image, (cx, cy), (int)(radius), (0, 255, 255), 2)

    return contours_image


def get_contour_centroid(contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    return cx, cy


def process_frame(rgb_frame, flip_image):
    global color_lower_bound, color_upper_bound
    global is_bkg_black

    # blur
    rgb_frame_blur = cv2.GaussianBlur(rgb_frame, (5, 5), 0)
    # detect color
    mask = filter_color(rgb_frame_blur, color_lower_bound, color_upper_bound)
    # find contours
    contours = find_contours(mask)
    # draw contours
    contoured_frame = draw_ball_contours(rgb_frame, contours, is_bkg_black)

    # mirror for display
    if flip_image:
        contoured_frame = cv2.flip(contoured_frame, 1)

    return contoured_frame


def main():

    frame_wait_ms = 50

    # Set video_source = filename
    # or = 0 to use the live camera as source

    # video_source = 0
    video_source = "tennis-ball-video.mp4"

    video_capture = cv2.VideoCapture(video_source)

    frame_counter = 0
    while True:
        ret, rgb_frame = video_capture.read()

        if video_source == 0:
            pass
        else:
            # If video source is a file, loop indefinitely
            frame_counter += 1
            # When the last frame is reached
            # Reset the capture and the frame_counter
            if frame_counter == video_capture.get(cv2.CAP_PROP_FRAME_COUNT):
                frame_counter = 0
                video_capture.set(cv2.CAP_PROP_POS_FRAMES, frame_counter)

        flip_image = video_source == 0
        processed_frame = process_frame(rgb_frame, flip_image)
        cv2.imshow("Frame", processed_frame)

        # stop if any key is pressed
        if cv2.waitKey(frame_wait_ms) > -1:  # & 0xFF == ord('q'):
            break

    video_capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
