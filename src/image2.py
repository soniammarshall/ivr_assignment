#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
import vision as vis
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image2_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize publishers to publish target distance estimates for y and z
    self.target_x_pub = rospy.Publisher("target_x_estimate", Float64, queue_size=10)
    self.target_z_pub = rospy.Publisher("target_z_estimate2", Float64, queue_size=10)
    # initialize publishers to publish end effector position estimates for x and z
    self.end_effector_x_pub = rospy.Publisher("end_effector_x", Float64, queue_size=10)
    self.end_effector_z_pub = rospy.Publisher("end_effector_z2", Float64, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.target_history = [0.0, 0.0]


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Receive the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Color masks (BGR)
    yellow_mask = cv2.inRange(self.cv_image2, (0, 170, 170), (80, 255, 255))
    red_mask = cv2.inRange(self.cv_image2, (0, 0, 100), (80, 80, 255))

    orange_mask = cv2.inRange(self.cv_image2, (75, 100, 125), (90, 180, 220))
    orange_mask = cv2.erode(orange_mask, vis.erode_dilate_kernel, iterations=1)
    orange_mask = cv2.dilate(orange_mask, vis.erode_dilate_kernel, iterations=1)

    sphere_position = vis.find_target(orange_mask, vis.sphere_template, self.target_history)
    base_frame = vis.detect_color(yellow_mask)
    sphere_relative_distance = np.absolute(sphere_position - base_frame)
    x_distance = Float64()
    x_distance.data = vis.to_meters_ratio_img2 * sphere_relative_distance[0]

    # Visualize movement of target
    # x_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), (sphere_position[0], base_frame[1]), color=(255, 255, 255))
    # z_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), (base_frame[0], sphere_position[1]), color=(255, 255, 255))
    # center_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), (sphere_position[0], sphere_position[1]), color=(255, 255, 255))
    # cv2.imshow('Visualization Target ZX', orange_mask)

    # x, z position of the end effector (the centre of the red sphere)
    end_effector_position = np.absolute(vis.detect_color(red_mask) - base_frame)
    end_effector_x = Float64()
    end_effector_z = Float64()
    end_effector_x.data = vis.to_meters_ratio_img2 * end_effector_position[0]
    end_effector_z.data = vis.to_meters_ratio_img2 * end_effector_position[1]

    #cv2.imshow('Original Cam ZX', self.cv_image2)
    cv2.waitKey(3)

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.target_x_pub.publish(x_distance)
      self.end_effector_x_pub.publish(end_effector_x)
      self.end_effector_z_pub.publish(end_effector_z)
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  image2_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


