/*********************************************************************
 * Compiler:         >gcc 4.6.3
 *
 * Company:          Chalmers University of Technology
 *
 * Author:           Martin Asplund
 *
 * Compatibility:    Ubuntu 18.04 64bit (ros melodic)
 *
 * Software Version: V0.1
 *
 * Created:          13.12.2021
 *
 * Comment:          *add later
 *
 ********************************************************************/

/*********************************************************************
 * STD INCLUDES
 ********************************************************************/
#include <fstream>
#include <iostream>
#include <pthread.h>

/*********************************************************************
 * ROS INCLUDES
 ********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/*********************************************************************
 * CUSTOM INCLUDES
 * ******************************************************************/
// #include <the_drink_pack/DrinkClass.h> // Might want to have a class
#include <robotic_pusher/getVelocity.h>
#include <robotic_pusher/getWeightType.h>
#include <robotic_pusher/spawnObject.h>

using namespace std;

#define object_name "random"

/*  Global parameters for the clients   */
ros::ServiceClient client_spawn;
ros::ServiceClient client_weight;

robotic_pusher::spawnObject object;
robotic_pusher::getWeightType srv;

float get_action() {
  // Temporary
  return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

bool get_velocity(robotic_pusher::spawnObject::Request &req,
                  robotic_pusher::spawnObject::Response &res) {

  // Pretty dumb check but why not
  if (!req.get_velocity) {
    return false;
  }

  /*  Call the service to spawn a object  */
  if (client_spawn.call(object)) {
    ROS_INFO_STREAM("Response: " << object.response.reply);
  } else {
    ROS_ERROR_STREAM("Failed to spawn a random object");
    return false;
  }

  string object_weight;
  /*  Call the service to get the weight of the spawned object  */
  if (client_weight.call(srv)) {
    ROS_INFO_STREAM("Response: " << srv.response.weight_type);
    object_weight = srv.response.weight_type;
  } else {
    ROS_ERROR_STREAM("Failed to get the weight from the object");
    return false;
  }

  // Change this function to get different actions
  res.impact_velocity = get_action();

  return true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "get_action_node", ros::init_options::AnonymousName);

  ROS_INFO_STREAM("Starting up action node...");

  ros::NodeHandle n;
  ros::Rate r(60);

  // Clients
  client_spawn = n.serviceClient<robotic_pusher::spawnObject>("spawn_object");
  client_weight = n.serviceClient<robotic_pusher::getWeightType>("get_weight");

  srv.request.get_weight_class = true;
  object.request.model_name = object_name;

  // Service
  ros::ServiceServer service =
      n.advertiseService("get_velocity", &get_velocity);

  ROS_INFO_STREAM("Service is up...");

  ros::spin();

  return 0;
}