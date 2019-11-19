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
