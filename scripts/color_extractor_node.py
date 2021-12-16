#!/usr/bin/env python
import rospy
from std_msgs.msg import String, UInt8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from robotic_pusher.srv import *

class color_extractor:

    def __init__(self):

        self.subscriber = rospy.Subscriber("/xtion/rgb/image_color", Image, self.callback)
        self.service = rospy.Service('/robotic_pusher/getColor', getColor, self.returnColor)

        self.bridge = CvBridge()
        self.color_value = []

        self.x = 200 # default value
        self.y = 200 # default value

        if rospy.has_param('/color_extraction_pixel_location/x'): #If params are loaded, read them
            self.x = rospy.get_param('/color_extraction_pixel_location/x')
        if rospy.has_param('/color_extraction_pixel_location/y'):
            self.y = rospy.get_param('/color_extraction_pixel_location/y')
        
    
    def callback(self, img_msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        rgb = cv2_img[self.x, self.y, 0:3]
        self.color_value = rgb

        rospy.loginfo("Pixel: [" + str(self.x) + ", " + str(self.y) + "], Val: "  + str(rgb) )
    
    def decodeRGB(self):
        """
        Gazebo/Red : [  2   2 124]
        
        """
        color = str(self.color_value) # TODO: implement dictionary between rgb values and color names
        return color

    def returnColor(self, req):
        return getColorResponse(self.decodeRGB())


def main(args):
    extractor = color_extractor()
    rospy.init_node('color_extractor_node')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

