#!/usr/bin/env python
from __future__ import print_function

import sys
import cv2
import rospy
import numpy as np
import vision as vis
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class blobs_estimator:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('blob_estimation', anonymous=True)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.image1_callback)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.image2_callback)
        # initialize a publisher to publish position of blobs
        self.blob_pub = rospy.Publisher("/blobs_pos", Float64MultiArray, queue_size=10)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        self.blobs = Float64MultiArray()
        self.blobs_history = np.array([0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0])

    def image1_callback(self, data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Color masks (BGR)
        yellow_mask = cv2.inRange(self.cv_image1, (0, 170, 170), (80, 255, 255))
        green_mask = cv2.inRange(self.cv_image1, (0, 100, 0), (80, 255, 80))
        red_mask = cv2.inRange(self.cv_image1, (0, 0, 100), (80, 80, 255))

        # update y and z of blobs
        # get the blob positions relative to the blue blob (blue at (0,0,0))
        if len(self.blobs.data) == 0:
            new_blobs = self.blobs_history
        else:
            new_blobs = self.blobs.data
            self.blobs_history = new_blobs

        base_frame = vis.detect_color(yellow_mask)
        new_green = base_frame - vis.detect_color(green_mask)
        new_blobs[7] = vis.to_meters_ratio_img1 * new_green[0]
        new_blobs[8] = vis.to_meters_ratio_img1 * new_green[1]

        new_red = base_frame - vis.detect_color(red_mask)
        new_blobs[10] = vis.to_meters_ratio_img1 * new_red[0]
        new_blobs[11] = vis.to_meters_ratio_img1 * new_red[1]

        self.blobs.data = new_blobs
        self.blob_pub.publish(self.blobs)

        print("YELLOW:{}, BLUE:{}, GREEN:{}, RED:{}".format([new_blobs[0], new_blobs[1], new_blobs[2]], [new_blobs[3], new_blobs[4], new_blobs[5]], [new_blobs[6], new_blobs[7], new_blobs[8]], [new_blobs[9], new_blobs[10], new_blobs[11]]), end='\r')


    def image2_callback(self, data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Color masks (BGR)
        yellow_mask = cv2.inRange(self.cv_image2, (0, 170, 170), (80, 255, 255))
        green_mask = cv2.inRange(self.cv_image2, (0, 100, 0), (80, 255, 80))
        red_mask = cv2.inRange(self.cv_image2, (0, 0, 100), (80, 80, 255))

        # update x of blobs
        # get the blob positions relative to the blue blob (blue at (0,0,0))
        if len(self.blobs.data) == 0:
            new_blobs = self.blobs_history
        else:
            new_blobs = self.blobs.data
            self.blobs_history = new_blobs

        base_frame = vis.detect_color(yellow_mask)
        new_green = base_frame - vis.detect_color(green_mask)
        new_blobs[6] = vis.to_meters_ratio_img2 * new_green[0]

        new_red = base_frame - vis.detect_color(red_mask)
        new_blobs[9] = vis.to_meters_ratio_img2 * new_red[0]

        self.blobs.data = new_blobs
        self.blob_pub.publish(self.blobs)


# call the class
def main(args):
    blobs_estimator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
