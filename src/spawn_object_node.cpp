#include "robotic_pusher/spawnObject.h"
#include "ros/ros.h"

bool spawn(robotic_pusher::spawnObject::Request &req,
           robotic_pusher::spawnObject::Response &res) {

  std::string model_name = req.model_name;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  res.reply = 0;
  if (ros::param::has("/spawn_position/x")) {
    ros::param::get("/spawn_position/x", x);
    ROS_INFO_STREAM("/spawn_position/x= " << x);
  } else {
    ROS_ERROR_STREAM("param: /spawn_position/x, doesn't exist");
    return true;
  }

  if (ros::param::has("/spawn_position/y")) {
    ros::param::get("/spawn_position/y", y);
    ROS_INFO_STREAM("/spawn_position/y= " << y);
  } else {
    ROS_ERROR_STREAM("param: /spawn_position/y, doesn't exist");
    return true;
  }

  if (ros::param::has("/spawn_position/z")) {
    ros::param::get("/spawn_position/z", z);
    ROS_INFO_STREAM("/spawn_position/z= " << z);
  } else {
    ROS_ERROR_STREAM("param: /spawn_position/z, doesn't exist");
    return true;
  }

  ROS_INFO("HEJSAN");

  std::string position = "-x " + std::to_string(x) + " -y " +
                         std::to_string(y) + " -z " + std::to_string(z);

  std::string directory_to_models =
      "/home/user/simulation_ws/src/tiago_simulation/tiago_gazebo/models/";
  std::string model_path =
      directory_to_models + model_name + "/" + model_name + ".sdf";

  std::string spawn_message = "rosrun gazebo_ros spawn_model -file " +
                              model_path + " -sdf " + position +
                              " -model my_object";

  ROS_INFO("%s", spawn_message.c_str());
  
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