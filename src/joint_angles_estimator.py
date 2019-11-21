#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from scipy.optimize import minimize, least_squares

class joint_angles_estimator:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('joint_angles_estimation', anonymous=True)
        # initialize a subscriber to get position of blobs
        self.blob_sub = rospy.Subscriber("/blobs_pos", Float64MultiArray, self.callback)
        self.blobs_history = np.array([0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0])
        # initialize a publisher to publish measured joint angles
        self.joint_angles_pub = rospy.Publisher("/joint_angles", Float64MultiArray, queue_size=10)
        self.joint_angles = Float64MultiArray()
        self.joint_angles_history = np.array([0.5, 0.5, 0.5, 0.5])

    # Receive target data from blob estimator, calculate joint angles and publish them
    def callback(self, blobs):
        if len(blobs.data) == 0:
            received_blobs = self.blobs_history
        else:
            received_blobs = blobs.data
            self.blobs_history = received_blobs

        joint_angles = self.measure_angles()
        self.joint_angles.data = joint_angles

        print("THETA 1:{0:.2f}, THETA 2:{1:.2f}, THETA 3:{2:.2f}".format(joint_angles[0], joint_angles[1], joint_angles[2]), end='\r')
        self.joint_angles_pub.publish(self.joint_angles)


    def error_fct(self, theta):
        s1 = np.sin(theta[0])
        c1 = np.cos(theta[0])
        s2 = np.sin(theta[1])
        c2 = np.cos(theta[1])
        s3 = np.sin(theta[2])
        c3 = np.cos(theta[2])

        blue = np.array([self.blobs_history[3], self.blobs_history[4], self.blobs_history[5]])
        green = np.array([self.blobs_history[6], self.blobs_history[7], self.blobs_history[8]])

        blue_to_green = green - blue
        # print("blue: {},  green: {}, blue_to_green: {}".format(blue, green, blue_to_green), end='\r')
        blue_to_green[0] = - blue_to_green[0]
        blue_to_green = blue_to_green / np.linalg.norm(blue_to_green)
        # print("blue: {},  green: {}, blue_to_green: {}".format(blue, green, blue_to_green), end='\r')

        # return error which is to be minimized

        # to be used with first set of DH params
        # return abs(c1 * c3 + s1 * s2 * s3 - blue_to_green[0]) + abs(s1 * c3 - c1 * s2 * s3 - blue_to_green[1]) + abs(c2 * s3 - blue_to_green[2])
        # to be used with second set of DH params
        return abs(c1 * c2 * c3 - s1 * s3 - blue_to_green[0]) + abs(s1 * c2 * c3 + c1 * s3 - blue_to_green[1]) + abs(-(s2 * c3) - blue_to_green[2])

    def measure_angles(self):
        # Descends in the angle space towards the minimum error, minimizing the error function and finding theta 1, 2 and 3
        # joint_angles = minimize(self.error_fct, np.array([0.5, 0.5, 0.5]), method='nelder-mead', options={'xtol': 1e-6})
        joint_angles = least_squares(self.error_fct, np.array([0.5, 0.5, 0.3]), bounds=(0, 1))
        # self.joint_angles_history[:-1]
        return joint_angles.x


# call the class
def main(args):
    ja = joint_angles_estimator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
