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
        self.service = rospy.Service('/robotic_pusher/get_color', getColor, self.returnColor)

        self.bridge = CvBridge()
        self.color_value = []

        self.x = 200 # default value
        self.y = 200 # default value

        if rospy.has_param('/color_extraction_pixel_location/x'): #If params are loaded, read them
            self.x = rospy.get_param('/color_extraction_pixel_location/x')
        if rospy.has_param('/color_extraction_pixel_location/y'):
            self.y = rospy.get_param('/color_extraction_pixel_location/y')
        
        self.color_value_list = [[102, 102, 102], [45, 45, 45], [23, 23, 23], [10, 10, 10], [0, 0, 0], [2, 2, 214], [7, 27, 90], 
                                [124, 2, 2], [71, 45, 13], [2, 124, 2], [6, 62, 122], [0, 122, 122], [29, 135, 138], [0, 85, 85], 
                                [2, 25, 41], [102, 0, 102], [102, 102, 0], [51, 0, 34]]
        self.color_name_list = ["White", "Grey", "DarkGrey", "FlatBlack", "Black", "Red", "BrightRed", 
                               "Blue", "SkyBlue", "Green", "Orange", "Yellow", "ZincYellow", "DarkYellow", 
                               "Gold", "Purple", "Turqoise", "Indigo"]
      
        """
        Color / bgr8
        Gazebo/White:       [102 102 102]  idx: 0
        Gazebo/Grey:        [45 45 45]
        Gazebo/DarkGrey:    [23 23 23]
        Gazebo/FlatBlack:   [10 10 10]
        Gazebo/Black:       [0 0 0]

        Gazebo/Red:         [  2   2 124]
        Gazebo/RedBright    [ 7 27 89]
        Gazebo/Blue:        [124   2   2]
        Gazebo/SkyBlue:     [71 45 13]
        Gazebo/Green:       [  2 124   2]

        Gazebo/Orange:      [  6  62 122]
        Gazebo/Yellow:      [  0 122 122]
        Gazebo/ZincYellow:  [ 29 135 138]
        Gazebo/DarkYellow:  [ 0 85 85]
        Gazebo/Gold:        [ 2 25 41]
        
        Gazebo/Purple:      [102   0 102]
        Gazebo/Turquoise:   [102 102   0]
        Gazebo/Indigo:      [51  0 34]
        """
        
    def callback(self, img_msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        bgr8 = cv2_img[self.x, self.y, 0:3]
        self.color_value = bgr8

        rospy.loginfo("Pixel: [" + str(self.x) + ", " + str(self.y) + "], bgr8: "  + str(bgr8) )

    
    def decodeRGB(self):        
        sensor_color = np.array(self.color_value)
        distances = np.sqrt(np.sum((self.color_value_list-sensor_color)**2,axis=1))
        rospy.loginfo(np.array2string(distances))
        color_idx = np.nanargmin(distances)
        return self.color_name_list[color_idx]

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

