#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray

class forward_kinematics:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('forward_kinematics', anonymous=True)
        # initialize a subscriber to get position of blobs
        self.blob_sub = rospy.Subscriber("/blobs_pos", Float64MultiArray, self.callback)
        self.blobs_history = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.joints = np.array([0.0, 0.0, 0.0, 0.0])

    # TODO when we have calculated joint angles change the subscribers to receive joint angles
    # TODO publish forward kinematics result to a topic
    def callback(self, blobs):
        if len(blobs.data) == 0:
            received_blobs = self.blobs_history
        else:
            received_blobs = blobs.data
            self.blobs_history = received_blobs

        print("x: {}, y:{}, z:{}".format(received_blobs[9], received_blobs[10], received_blobs[11]))
        end_effector = self.calculate_fk(self.joints)
        print("FK x: {}, y: {}, z: {}".format(end_effector[0], end_effector[1], end_effector[2]))

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