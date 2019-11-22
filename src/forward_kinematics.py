#!/usr/bin/env python
from __future__ import print_function

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
        # initialize a publisher to publish the end effector position calculated by FK
        self.end_effector_pub = rospy.Publisher("/end_effector_position", Float64MultiArray, queue_size=10)
        self.end_effector_position = Float64MultiArray()
        self.blobs_history = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421
        # input the joint angles here for which you want to calculate FK
        self.joints = np.array([0.0, self.pi/2, 0.0, 0.0])


    def callback(self, blobs):
        if len(blobs.data) == 0:
            received_blobs = self.blobs_history
        else:
            received_blobs = blobs.data
            self.blobs_history = received_blobs

        # print("Vision\tx: {}, y:{}, z:{}".format(received_blobs[9], received_blobs[10], received_blobs[11]), end='\r')
        end_effector = self.calculate_fk(self.joints)
        print("FK\tx: {}, y: {}, z: {}".format(end_effector[0], end_effector[1], end_effector[2]), end='\r')
        self.end_effector_position.data = end_effector
        self.end_effector_pub.publish(self.end_effector_position)
        # end_effector_new = self.calculate_fk_version2_wrong(self.joints)
        # print("New FK\tx: {}, y: {}, z: {}".format(end_effector_new[0], end_effector_new[1], end_effector_new[2]))


    def calculate_fk(self, joints):
        s1 = np.sin(joints[0])
        c1 = np.cos(joints[0])
        s2 = np.sin(joints[1])
        c2 = np.cos(joints[1])
        s3 = np.sin(joints[2])
        c3 = np.cos(joints[2])
        s4 = np.sin(joints[3])
        c4 = np.cos(joints[3])
        x_e = (-c1*s3+s1*s2*c3)*(2*c4+3)+2*s4*(-s1*s2*s3-c1*c3)
        y_e = (-s1*s3-c1*s2*c3)*(2*c4+3)+2*s4*(-c1*s2*s3+s1*c3)
        z_e = c2 * c3 * (2 * c4 + 3) + 2 * (- s4 * c2 * s3 + 1)
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