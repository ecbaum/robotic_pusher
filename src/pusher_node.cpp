#include <robotic_pusher/getVelocity.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <exception>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <string>
#include "gazebo_msgs/GetModelState.h"

// TODO: Adjust velocity mapping and threshhold for moving criteria

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JointClient;

// helper function, constrains the the variable x between the lower value a and higher value b
float constrain(float &x, const float &a, const float &b) {
  assert(a<b);
  if (x < a) {
    x = a;
  } else if (x > b) {
    x = b;
  }
  return x;
}

// callback function for pusher_service with /gazebo/get_model_state client
bool push_object_get_distance(robotic_pusher::getVelocity::Request  &req, robotic_pusher::getVelocity::Response &res){
    // Get velocity and check whether in range [0,1]
    float velocity = req.impact_velocity;
    if(velocity <=1 && velocity >=0){
        ROS_INFO("Velocity I got is %f, huiiiiii", (double)velocity);
    }
    else {
        ROS_ERROR("Velocity has to be in interval [0,1]");
        return false;
    }

    /****************************************************************/
    /***********************Execute THE SLAPPP***********************/
    /****************************************************************/
    JointClient armClient("/arm_controller/follow_joint_trajectory", true);
    // wait for the action server to come up
    armClient.waitForServer();
    control_msgs::FollowJointTrajectoryGoal arm_goal;
    arm_goal.trajectory.joint_names.push_back("arm_1_joint");
    arm_goal.trajectory.joint_names.push_back("arm_2_joint");
    arm_goal.trajectory.joint_names.push_back("arm_3_joint");
    arm_goal.trajectory.joint_names.push_back("arm_4_joint");
    arm_goal.trajectory.joint_names.push_back("arm_5_joint");
    arm_goal.trajectory.joint_names.push_back("arm_6_joint");
    arm_goal.trajectory.joint_names.push_back("arm_7_joint");
    arm_goal.trajectory.points.resize(3);

    // Start Position
    int index = 0;
    // Positions
    arm_goal.trajectory.points[index].positions.resize(7);
    arm_goal.trajectory.points[index].positions[0] = constrain(velocity, 0.07, 1.57);
    arm_goal.trajectory.points[index].positions[1] = 0.0;
    arm_goal.trajectory.points[index].positions[2] = 1.5;
    arm_goal.trajectory.points[index].positions[3] = constrain(velocity, 0.0, 2.29);
    arm_goal.trajectory.points[index].positions[4] = -1.57;
    arm_goal.trajectory.points[index].positions[5] = 0.0;
    arm_goal.trajectory.points[index].positions[6] = 0.0;
    // To be reached 2 second after starting along the trajectory
    arm_goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

    // Impact Position
    index++;
    arm_goal.trajectory.points[index] = arm_goal.trajectory.points[0];
    // Positions
    arm_goal.trajectory.points[index].positions[0] = 1.57;
    arm_goal.trajectory.points[index].positions[3] = 0.0;
    // Velocities
    arm_goal.trajectory.points[index].velocities.resize(7);
    arm_goal.trajectory.points[index].velocities[0] = velocity / 2;
    arm_goal.trajectory.points[index].velocities[3] = velocity / 2;
    // To be reached 4 second after starting along the trajectory
    arm_goal.trajectory.points[index].time_from_start = ros::Duration(3.0);

    // End Position
    index++;
    arm_goal.trajectory.points[index] = arm_goal.trajectory.points[0];
    // Positions
    arm_goal.trajectory.points[index].positions[0] = 1.75;
    arm_goal.trajectory.points[index].positions[3] = -0.15;
    // To be reached 6 second after starting along the trajectory
    arm_goal.trajectory.points[index].time_from_start = ros::Duration(4.0);

    // while(ros::ok()){
    arm_goal.trajectory.header.stamp = ros::Time::now();
    armClient.sendGoal(arm_goal);
    ROS_INFO("Starting Movement now");
    armClient.waitForResult();
    // Wait for trajectory execution
    if (armClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Movement complete");
    } else {
        ROS_ERROR("Failed to do push movement");
        return false;
    }

    /****************************************************************/
    /***********************Get the cube position after slapp********/
    /****************************************************************/
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = "push_box";
    // Check if we are still moving
    bool moving = true;
    double prev_position = 0;
    while(moving){
        if (client.call(srv))
        {
            double position = srv.response.pose.position.x;
            if(position <= prev_position+0.1 && position >= prev_position-0.1){
                moving = false;
            }
            else {
                prev_position = position;
            }
            ROS_INFO("current x-position: %f", position);
            ros::Duration(1).sleep();
        }
        else
        {
            ROS_ERROR("Failed to call service /gazebo/get_model_state");
            return false;
        }
    }
    // Give final cube position as response
    if (client.call(srv))
        {
            ROS_INFO("Final position: x: %f, y: %f, y: %f", srv.response.pose.position.x, srv.response.pose.position.y, srv.response.pose.position.z);
            res.position = srv.response.pose.position;
        }
        else
        {
            ROS_ERROR("Failed to call service /gazebo/get_model_state");
            return false;
        }
    return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pusher_node");
  ros::NodeHandle nh;

  /****************************************************************/
  /***********************Call pusher_service**********************/
  /****************************************************************/
  ros::ServiceServer service = nh.advertiseService("pusher_service", push_object_get_distance);
  ros::spin();
  return 0;
}