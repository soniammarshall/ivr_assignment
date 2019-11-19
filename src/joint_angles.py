#!/usr/bin/env python
import os

import roslib
import sys
import rospy
import cv2
import numpy as np
import vision as vis
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class joint_angles:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('joint_angles_estimation', anonymous=True)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.target_y_sub = rospy.Subscriber("/target_y_estimate", Float64, self.callback_y_update)
        self.target_x_sub = rospy.Subscriber("/target_x_estimate", Float64, self.callback_x_update)
        self.target_y = Float64()
        self.target_x = Float64()
        # initialize publishers to publish target distance estimates for y and z
        # self.target_y_pub = rospy.Publisher("target_y_estimate", Float64, queue_size=10)
        # self.target_z_pub = rospy.Publisher("target_z_estimate", Float64, queue_size=10)

    # Receive target data from both cameras, process it, and publish
    def callback_y_update(self, data):
        self.target_y = data
        print(np.rad2deg(np.arctan2(self.target_x.data, self.target_y.data)))

    def callback_x_update(self, data):
        self.target_x = data
        print(np.rad2deg(np.arctan2(self.target_x.data, self.target_y.data)))


# call the class
def main(args):
    ja = joint_angles()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
