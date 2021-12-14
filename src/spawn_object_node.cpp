#include "robotic_pusher/spawnObject.h"
#include "ros/ros.h"

bool spawn(robotic_pusher::spawnObject::Request &req,
           robotic_pusher::spawnObject::Response &res) {

    std::string model_name = req.model_name;
    std::string position = req.position;

    std::string directory_to_models = "/home/user/simulation_ws/src/tiago_simulation/tiago_gazebo/models/";
    std::string model_path = directory_to_models + model_name + "/" + model_name + ".sdf";
    std::string spawn_message = "rosrun gazebo_ros spawn_model -file " + model_path + " -sdf " + position + " -model my_object";

    system("rosservice call gazebo/delete_model '{model_name: my_object}'");
    system(spawn_message.c_str());
    
    res.reply = 1;

    ROS_INFO("Spawning object: %s", model_name.c_str());
    return true;
}

int main(int argc, char **argv) {
    ROS_INFO("spawn_object_node started");
    ros::init(argc, argv, "spawn_object_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("spawn_object", spawn);

    ros::spin();

    return 0;
}