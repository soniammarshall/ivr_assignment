#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
import vision as vis
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    orange_mask = cv2.inRange(self.cv_image2, (75, 100, 125), (90, 180, 220))
    sphere_position = vis.find_target(orange_mask, vis.sphere_template)
    print(vis.to_meters_ratio * sphere_position[1], vis.to_meters_ratio * sphere_position[0])  # distance of Z and of Y

    ''' Visualize movement
    yellow_mask = cv2.inRange(self.cv_image2, (0, 170, 170), (80, 255, 255))
    base_frame = vis.detect_color(yellow_mask)
    x_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), (sphere_position[0], base_frame[1]), color=(255, 255, 255))
    y_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), (base_frame[0], sphere_position[1]), color=(255, 255, 255))
    cv2.imshow('Visualization', orange_mask)
    '''

    # cv2.imshow('Original Cam ZY', self.cv_image2)
    cv2.waitKey(3)

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


