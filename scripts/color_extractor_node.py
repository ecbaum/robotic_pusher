#!/usr/bin/env python
import rospy
from std_msgs.msg import String, UInt8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2


class color_extractor:

  def __init__(self):
    self.pub = rospy.Publisher("extracted_color", String, queue_size=10)
    self.sub = rospy.Subscriber("/xtion/rgb/image_color", Image, self.callback)
    self.bridge = CvBridge()

  def callback(self, img_msg):
    try:
        cv2_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)


    if rospy.has_param('/color_extraction_pixel_location/x'):
        x = rospy.get_param('/color_extraction_pixel_location/x')
    else:
        x = 200 # default value
    if rospy.has_param('/color_extraction_pixel_location/y'):
        y = rospy.get_param('/color_extraction_pixel_location/y')
    else:
        y = 200 # default value
    

    rgb = cv2_img[x, y, 0:3]

    rospy.loginfo("rgb: " + str(rgb))
    
    self.pub.publish(str(rgb))

def main(args):
    extractor = color_extractor()
    rospy.init_node('color_extractor_node')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

