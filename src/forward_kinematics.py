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


class forward_kinematics:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('forward_kinematics', anonymous=True)
        # initialize subscribers to recieve messages from topics with end effector position
        self.end_effector_subx = rospy.Subscriber("/end_effector_x", Float64, self.callback1)
        self.end_effector_suby = rospy.Subscriber("/end_effector_y", Float64, self.callback2)
        self.end_effector_subz = rospy.Subscriber("/end_effector_z", Float64, self.callback3)
        self.end_effector_x = 0.0
        self.end_effector_y = 0.0
        self.end_effector_z = 0.0
        self.joints = np.array([0, 0, 0, 0])

    # TODO when we have calculated joint angles change the subscribers to receive joint angles
    # TODO publish forward kinematics result to a topic
    def callback1(self, msg):
        self.end_effector_x = msg.data
        print("x: {}".format(self.end_effector_x))
        end_effector = self.calculate_fk(self.joints)
        print("FK x: {}, y: {}, z: {}".format(end_effector[0], end_effector[1], end_effector[2]))

    def callback2(self, msg):
        self.end_effector_y = msg.data
        print("y: {}".format(self.end_effector_y))

    def callback3(self, msg):
        self.end_effector_z = msg.data
        print("z: {}".format(self.end_effector_z))


    def calculate_fk(self, joints):
        x_e = np.sin(joints[0]) * np.sin(joints[1]) * (2 * np.sin(joints[2] + joints[3]) + 3 * np.sin(joints[2])) + np.cos(joints[0]) * (2 * np.cos(joints[2] + joints[3]) + 3 * np.cos(joints[2]))
        y_e = np.sin(joints[0]) * (2 * np.cos(joints[2] + joints[3]) + 3 * np.cos(joints[2])) - np.cos(joints[0]) * np.sin(joints[1]) * (2 * np.sin(joints[2] + joints[3]) + 3 * np.sin(joints[2]))
        z_e = np.cos(joints[1]) * np.sin(joints[2]) * (2 * np.cos(joints[3] + 3)) + 2 * (np.sin(joints[3]) * np.cos(joints[1]) * np.cos(joints[2]) + 1)
        end_effector = np.array([x_e, y_e, z_e])
        return end_effector


# call the class
def main(args):
    fk = forward_kinematics()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)