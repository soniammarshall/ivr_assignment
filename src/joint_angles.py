#!/usr/bin/env python

import sys
import rospy
import numpy as np
from std_msgs.msg import Float64
from scipy.optimize import minimize

class joint_angles_estimator:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('joint_angles_estimation', anonymous=True)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.target_y_sub = rospy.Subscriber("/target_y_estimate", Float64, self.callback_y_update)
        self.target_x_sub = rospy.Subscriber("/target_x_estimate", Float64, self.callback_x_update)
        self.target_y = Float64()
        self.target_x = Float64()

    def error_fct(self, theta):
        s1 = np.sin(theta[0])
        c1 = np.cos(theta[0])
        s2 = np.sin(theta[1])
        c2 = np.cos(theta[1])
        s3 = np.sin(theta[2])
        c3 = np.cos(theta[2])

        blue_to_green = self.x_measured[2] - self.x_measured[1]
        blue_to_green = blue_to_green / np.linalg.norm(blue_to_green)

        # return error which is to be minimized
        return abs(c1 * s3 + s1 * s2 * c3 - blue_to_green[0]) + abs(s1 * s3 - c1 * s2 * c3 - blue_to_green[1]) + abs(c2 * c3 - blue_to_green[2])

    def measure_angle(self):
        # Descends in the angle space towards the minimum error, minimizing the error function and finding theta 1, 2 and 3
        theta_num = minimize(self.error_fct, self.q_d[:-1], method='nelder-mead', options={'xtol': 1e-6})
        theta_num = theta_num.x

        green_to_red = self.x_measured[3] - self.x_measured[2]
        green_to_red = green_to_red / np.linalg.norm(green_to_red)

        dd = self.roty(-theta_num[2]).dot(self.rotx(-theta_num[1]).dot(self.rotz(-theta_num[0]).dot(green_to_red)))

        theta4 = np.arctan2(-dd[1], dd[2])
        theta_num = np.append(theta_num, theta4)
        return theta_num

    # Receive target data from both cameras, process it, and publish
    def callback_y_update(self, data):
        self.target_y = data
        print(np.rad2deg(np.arctan2(self.target_x.data, self.target_y.data)))

    def callback_x_update(self, data):
        self.target_x = data
        print(np.rad2deg(np.arctan2(self.target_x.data, self.target_y.data)))

# call the class
def main(args):
    joint_angles_estimator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
