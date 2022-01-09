#!/usr/bin/env python
import rospy
from std_msgs.msg import String, UInt8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from robotic_pusher.srv import *
from gazebo_msgs.srv import DeleteModel
import os
import random

class cube_spawner:

    def __init__(self):

        self.service = rospy.Service('robotic_pusher/spawn_cube', spawnObject, self.callback_srv)

        self.x = 1
        self.y = 0.27
        self.z = 1.14

        self.position = "-x " + str(self.x) + " -y " + str(self.y) + " -z " + str(self.z)

        self.spawn_msg_prefix = "rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/src/robotic_pusher/models/"
        self.spawn_msg_suffix = "/model.sdf" + " -sdf " + self.position + " -model push_cube"

        self.models = ["red_cube", "blue_cube", "green_cube", "yellow_cube"]
        self.models_training = ["red_cube", "blue_cube", "green_cube"]

        #self.delete_model_client()

    def delete_model_client(self):
        
        rospy.loginfo("Deleting model")
        rospy.loginfo("Waiting for service gazebo/delete_model")
        rospy.wait_for_service('gazebo/delete_model')
        response = 0
        try:
            delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
            resp1 = delete_model('push_cube')
            response = resp1.success
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
        return response
    
    def run_spawn_message(self, model_name, training):
        if model_name == 'random':
            if training:
                model_name = random.choice(self.models_training)
            else:
                model_name = random.choice(self.models)
        
        spawn_message = self.spawn_msg_prefix + model_name + self.spawn_msg_suffix
        rospy.loginfo("Spawning " + model_name)
        rospy.loginfo("Running '" + spawn_message + "'")
        self.object_spawned = True
        os.system(spawn_message)

    def callback_srv(self, req):

        response = self.delete_model_client()

        if response == 1:
            rospy.loginfo("Delete successful")
        else:
            rospy.loginfo("Delete not successful")
        
        rospy.loginfo("Model input: " + req.model_name)
        self.run_spawn_message(req.model_name, req.training)

        return 1
        
def main(args):
    rospy.loginfo("starting spawn_cube_node")
    spawner = cube_spawner()
    rospy.init_node('spawn_cube_node')
    rospy.loginfo("spawn_cube_node started")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

