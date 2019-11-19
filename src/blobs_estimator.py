#!/usr/bin/env python

import sys
import cv2
import rospy
import numpy as np
import vision as vis
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

class blobs_estimator:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('blob_estimation', anonymous=True)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.image1_callback)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.image2_callback)
        self.end_effector_y_pub = rospy.Publisher("/end_effector_y", Float64, queue_size=10)
        self.end_effector_z_pub = rospy.Publisher("/end_effector_z", Float64, queue_size=10)
        self.end_effector_x_pub = rospy.Publisher("/end_effector_x", Float64, queue_size=10)
        # initialize a publisher to publish position of blobs
        self.blob_pub2 = rospy.Publisher("/blob_pos",Float64MultiArray, queue_size=10)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        self.red_x = Float64()
        self.red_y = Float64()
        self.red_z = Float64()

    def image1_callback(self, data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Color masks (BGR)
        yellow_mask = cv2.inRange(self.cv_image1, (0, 170, 170), (80, 255, 255))
        red_mask = cv2.inRange(self.cv_image1, (0, 0, 100), (80, 80, 255))

        # y, z position of the end effector (the centre of the red sphere)
        base_frame = vis.detect_color(yellow_mask)
        end_effector_position = np.absolute(vis.detect_color(red_mask) - base_frame)
        self.red_y.data = vis.to_meters_ratio_img1 * end_effector_position[0]
        self.red_z.data = vis.to_meters_ratio_img1 * end_effector_position[1]

        print("Y={}, Z={}".format(self.red_y.data, self.red_z.data))
        self.end_effector_y_pub.publish(self.red_y)
        self.end_effector_z_pub.publish(self.red_z)


    def image2_callback(self, data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Color masks (BGR)
        yellow_mask = cv2.inRange(self.cv_image1, (0, 170, 170), (80, 255, 255))
        red_mask = cv2.inRange(self.cv_image1, (0, 0, 100), (80, 80, 255))

        # x position of the end effector (the centre of the red sphere)
        base_frame = vis.detect_color(yellow_mask)
        end_effector_position = np.absolute(vis.detect_color(red_mask) - base_frame)
        self.red_x.data = vis.to_meters_ratio_img2 * end_effector_position[0]

        print("X={}".format(self.red_x.data))
        self.end_effector_x_pub.publish(self.red_x)


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
