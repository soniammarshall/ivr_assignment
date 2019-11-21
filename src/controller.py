#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray

class controller:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('controller', anonymous=True)
        # initialize a subscriber to get position of blobs
        self.blob_sub = rospy.Subscriber("/blobs_pos", Float64MultiArray, self.callback1)
        # initialize a subscriber to get position of target
        self.target_sub = rospy.Subscriber("/target_position_estimate", Float64MultiArray, self.callback2)
        # initialize a publisher to publish joint angles to the robot
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.joint2 = Float64()
        self.joint2.data = 0.01
        self.increment = Float64()
        self.increment.data = 0.1


    def callback1(self, data):
        test = 0
        # TODO get blob positions


    def callback2(self, data):
        self.joint1 = Float64()
        self.joint1.data = 0.0
        # self.joint2= Float64()
        # self.joint2.data = 0.0
        self.joint3 = Float64()
        self.joint3.data = 0.0
        self.joint4 = Float64()
        self.joint4.data = 0.0
        print(self.joint2.data)
        self.robot_joint1_pub.publish(self.joint1)
        self.robot_joint2_pub.publish(self.joint2)
        self.robot_joint3_pub.publish(self.joint3)
        self.robot_joint4_pub.publish(self.joint4)
        if self.joint2.data >= 1.0 or self.joint2.data <= 0.0:
            self.increment.data = self.increment.data * -1
        self.joint2.data = self.joint2.data + self.increment.data


# call the class
def main(args):
    ctrl = controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
