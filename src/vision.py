#!/usr/bin/env python

import cv2
import numpy as np

to_meters_ratio_img1 = 0.04165762736367134  # Precomputed with pixel2meter
to_meters_ratio_img2 = 0.04322367230736768  # Precomputed with pixel2meter
sphere_template = cv2.imread("src/ivr_assignment/src/sphere.png", 0)
erode_dilate_kernel = np.ones((3, 3), np.uint8)

# Detecting the centre of a colored circle
def detect_color(mask):  # mask isolates the color in the image as a binary image
    # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)
    # Obtain the moments of the binary image
    M = cv2.moments(mask)
    # Calculate pixel coordinates for the centre of the blob
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])


# Calculate the conversion from pixel to meter
def pixel2meter(yellow_mask, blue_mask):
    yellow_joint = detect_color(yellow_mask)
    blue_joint = detect_color(blue_mask)
    # find the distance between two circles
    dist = np.sum((yellow_joint - blue_joint) ** 2)
    return 2 / np.sqrt(dist)  # link between yellow and blue is 2 meters


# Calculate the relevant joint angles from the image
def detect_joint_angles(yellow_mask, blue_mask, green_mask, red_mask, to_meters_ratio):
    # Obtain the centre of each coloured blob
    base_joint = to_meters_ratio * detect_color(yellow_mask)
    blue_joint = to_meters_ratio * detect_color(blue_mask)
    green_joint = to_meters_ratio * detect_color(green_mask)
    red_joint = to_meters_ratio * detect_color(red_mask)

    # Solve using trigonometry
    ja1 = np.arctan2(base_joint[0] - blue_joint[0], base_joint[1] - blue_joint[1])
    ja2 = np.arctan2(blue_joint[0] - green_joint[0], blue_joint[1] - green_joint[1]) - ja1
    ja3 = np.arctan2(green_joint[0] - red_joint[0], green_joint[1] - red_joint[1]) - ja2 - ja1

    return np.array([ja1, ja2, ja3])

def find_target(mask, template, target_history):
    # Build template
    w, h = template.shape[::-1]

    # Apply template Matching
    res = cv2.matchTemplate(mask, template, cv2.TM_CCOEFF)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

    # top_left = max_loc
    # bottom_right = (top_left[0] + w, top_left[1] + h)
    center = (max_loc[0] + w / 2, max_loc[1] + h / 2)

    # To detect the box:
    #box_template = cv2.imread("src/ivr_assignment/src/box.png", 0)
    if max_val < 6800000:
        return (target_history[0], target_history[1])

    target_history[0] = center[0]
    target_history[1] = center[1]
    return np.array([center[0], center[1]])