#!/usr/bin/env python

import sys
import cv2
import rospy
import numpy as np
import vision as vis
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

class target_estimator:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('target_estimation', anonymous=True)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.image1_callback)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.image2_callback)
        # initialize publishers to publish target distance estimates for y and z
        self.target_y_pub = rospy.Publisher("/target_y_estimate", Float64, queue_size=10)
        self.target_z_pub = rospy.Publisher("/target_z_estimate", Float64, queue_size=10)
        self.target_x_pub = rospy.Publisher("/target_x_estimate", Float64, queue_size=10)
        self.target_history = [0.0, 0.0, 0.0]
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        self.target_y = Float64()
        self.target_z = Float64()
        self.target_x = Float64()

    def find_target(self, mask, template, target_history, zy=False):
        # Build template
        w, h = template.shape[::-1]

        # Apply template matching
        res = cv2.matchTemplate(mask, template, cv2.TM_CCOEFF)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        # max_loc is the top left point of the match
        center = (max_loc[0] + w / 2, max_loc[1] + h / 2)

        # To detect the box:
        # box_template = cv2.imread("src/ivr_assignment/src/box.png", 0)
        if max_val < 6800000:
            if zy:
                return np.array([target_history[1], target_history[2]])
            else:
                return np.array([target_history[0], target_history[2]])

        if zy:
            target_history[1] = center[0]
        else:
            target_history[0] = center[0]
        target_history[2] = center[1]
        return np.array([center[0], center[1]])

    def image1_callback(self, data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # IMAGE 1
        # Color masks (BGR)
        orange_mask = cv2.inRange(self.cv_image1, (75, 100, 125), (90, 180, 220))
        kernel = np.ones((3, 3), np.uint8)
        orange_mask = cv2.erode(orange_mask, kernel, iterations=1)
        orange_mask = cv2.dilate(orange_mask, kernel, iterations=1)
        sphere_position = self.find_target(orange_mask, vis.sphere_template, self.target_history, True)
        base_frame = vis.yellow_blob_center_img1
        sphere_relative_distance = np.absolute(sphere_position - base_frame)

        # Visualize movement of target
        # y_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), (sphere_position[0], base_frame[1]), color=(255, 255, 255))
        # z_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), (base_frame[0], sphere_position[1]), color=(255, 255, 255))
        # center_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), (sphere_position[0], sphere_position[1]), color=(255, 255, 255))
        # cv2.imshow('Visualization Image 1, Target ZY', orange_mask)
        # cv2.imshow('Original Image 1, Target ZY', self.cv_image1)
        # cv2.waitKey(3)

        # Publish the results
        self.target_y.data = vis.to_meters_ratio_img1 * sphere_relative_distance[0]
        self.target_z.data = vis.to_meters_ratio_img1 * sphere_relative_distance[1]
        print("Y={}, Z={}".format(self.target_y.data, self.target_z.data))
        self.target_y_pub.publish(self.target_y)
        self.target_z_pub.publish(self.target_z)



    def image2_callback(self, data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        orange_mask = cv2.inRange(self.cv_image2, (75, 100, 125), (90, 180, 220))
        kernel = np.ones((2, 2), np.uint8)
        orange_mask = cv2.erode(orange_mask, kernel, iterations=1)
        orange_mask = cv2.dilate(orange_mask, kernel, iterations=1)
        sphere_position = self.find_target(orange_mask, vis.sphere_template, self.target_history, False)
        base_frame = vis.yellow_blob_center_img2
        sphere_relative_distance = np.absolute(sphere_position - base_frame)

        # Visualize movement of target
        # x_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), (sphere_position[0], base_frame[1]), color=(255, 255, 255))
        # z_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), (base_frame[0], sphere_position[1]), color=(255, 255, 255))
        # center_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), (sphere_position[0], sphere_position[1]), color=(255, 255, 255))
        # cv2.imshow('Visualization Image 2, Target ZX', orange_mask)
        # # cv2.imshow('Visualization Image 2, Yellow Blob ZX', yellow_mask)
        # cv2.waitKey(3)

        # Publish the results
        self.target_x = Float64()
        self.target_x.data = vis.to_meters_ratio_img2 * sphere_relative_distance[0]
        print("X={}".format(self.target_x.data))
        self.target_x_pub.publish(self.target_x)


# call the class
def main(args):
    target_estimator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
