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
        self.service = rospy.Service('robotic_pusher/get_color', getColor, self.returnColor)
        self.service_calibrate = rospy.Service('robotic_pusher/calibrate_color', calibrateColor, self.calibrate_callback    )

        self.bridge = CvBridge()
        self.color_value = []


        self.x = 0 # default value
        self.y = 0 # default value

        self.color_value_list = [[102, 102, 102], [0, 0, 0], [2, 2, 124],[124, 2, 2], [2, 124, 2], [0, 122, 122]]
        self.color_name_list = ["White", "Black", "Red", "Blue", "Green", "Yellow"]

        self.img = None

    def calibrate(self):
        img_local = self.img
        if img_local is not None:
            x_list = []
            y_list = []
            rospy.loginfo("Checking color of pixels")
            for x in range(1,360,10):
                for y in range(1,480,10):
                    color_value = img_local[x, y, 0:3]
                    color = self.decodeRGB(color_value)
                    #rospy.loginfo("x = " + str(x) + ", y = " + str (y) + ", color = " + str(color_value) + ", " + color)
                    if (color != 'White') and (color != 'Black'):                      
                        x_list.append(x)
                        y_list.append(y)
            if len(x_list) > 0:

                x_mean =  int(sum(x_list)/len(x_list))
                y_mean =  int(sum(y_list)/len(y_list))

                rospy.loginfo("Average pixel value with color: " + str(x_mean) + ", " + str(y_mean))
                self.x = x_mean
                self.y = y_mean
                return 1
            else:
                rospy.loginfo("Calibration error: No pixels of color found")
                return 0
        else:
            rospy.loginfo("Calibration error: no image loaded")
            return 0

    def calibrate_callback(self, req):
        response = self.calibrate()
        return response
        
    def callback(self, img_msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        bgr8 = cv2_img[self.x, self.y, 0:3]
        self.color_value = bgr8

        #rospy.loginfo("Pixel: [" + str(self.x) + ", " + str(self.y) + "], bgr8: "  + str(bgr8) )
        
        self.img = cv2_img

    
    def decodeRGB(self, color_value): 
        sensor_color = np.array(color_value)       
        distances = np.sqrt(np.sum((self.color_value_list-sensor_color)**2,axis=1))
        color_idx = np.nanargmin(distances)
        return self.color_name_list[color_idx]

    def returnColor(self, req):
        return getColorResponse(self.decodeRGB(self.color_value))


def main(args):
    extractor = color_extractor()
    rospy.init_node('color_extractor_node')
    rospy.loginfo("color_extractor_node started")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

