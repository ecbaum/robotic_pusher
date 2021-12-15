#!/usr/bin/env python
import rospy
from std_msgs.msg import String, UInt8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2


class color_extractor:

  def __init__(self):
    self.image_pub = rospy.Publisher("extracted_color",Image, queue_size=10)
    self.image_sub = rospy.Subscriber("/xtion/rgb/image_color",Image,self.callback)
    self.bridge = CvBridge()

  def callback(self, img_msg):
    try:
        cv2_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    x = 100 # pixels to extract from
    y = 50 

    rgb = cv2_img[x,y,0:3]

    rospy.loginfo("rgb: " + str(rgb))

if __name__ == '__main__':
    extractor = color_extractor()
    rospy.init_node('color_extractor_node', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")