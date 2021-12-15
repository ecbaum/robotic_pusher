#include "robotic_pusher/getVelocity.h"
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <exception>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <string>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    JointClient;
/*typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveClient;
void chatterCallback(const std_msgs::String::ConstPtr& msg)
    {
       ROS_INFO("I heard: [%s]", msg->data.c_str());
    }
*/
float velocity;

// helper function
float constrain(float &x, const float &a, const float &b) {
  if (x < a) {
    x = a;
  } else if (x > b) {
    x = b;
  }
  return x;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "run_test_push_node");
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(
          30.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }
  /****************************************************************/
  /***********************Move the torso up************************/
  /****************************************************************/
  ROS_INFO("First, grow up");
  JointClient torsoClient("/torso_controller/follow_joint_trajectory", true);
  // wait for the action server to come up
  torsoClient.waitForServer();
  control_msgs::FollowJointTrajectoryGoal torso_goal;
  torso_goal.trajectory.header.stamp = ros::Time::now();
  torso_goal.trajectory.joint_names.push_back("torso_lift_joint");
  torso_goal.trajectory.points.resize(1);
  torso_goal.trajectory.points[0].positions.resize(1);
  torso_goal.trajectory.points[0].positions[0] = 0.35;
  torso_goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
  torsoClient.sendGoal(torso_goal);
  ROS_INFO("Starting to move uuuup");
  torsoClient.waitForResult();
  if (torsoClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("...I am high");
  }
  // Wait task to be finished
  ROS_INFO("Wait shortly to make sure previous task is finished.");
  ros::Duration(1).sleep();

  /****************************************************************/
  /***********************Get the pushing velocity*****************/
  /****************************************************************/
  ros::ServiceClient pushclient =
      nh.serviceClient<robotic_pusher::getVelocity>("velocity_service");
  robotic_pusher::getVelocity srv;
  srv.request.get_velocity = true;
  if (pushclient.call(srv)) {
    velocity = srv.response.impact_velocity;
    ROS_INFO("Velocity I got is %f, huiiiiii", (double)velocity);
  } else {
    ROS_ERROR("Failed to get pushing velocity :(");
    return 1;
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
  arm_goal.trajectory.points[index].positions[0] =
      constrain(velocity, 0.07, 1.57);
  arm_goal.trajectory.points[index].positions[1] = 0.0;
  arm_goal.trajectory.points[index].positions[2] = 1.5;
  arm_goal.trajectory.points[index].positions[3] =
      constrain(velocity, 0.0, 2.29);
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
    return 1;
  }

  /* Might be needed if we have to move to the table

  // Wait between tasks
  ROS_INFO("Wait shortly to make sure previous task is finished.");
  ros::Duration(1).sleep();

  ROS_INFO("Starting moving to table");
  MoveClient client("move_base", true); // true -> don't need ros::spin()
  client.waitForServer();
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.stamp = ros::Time::now();
  // reference frame is map
  goal.target_pose.header.frame_id = "map";
  // set goal as 2D coordinate
  goal.target_pose.pose.position.x = 1;
  goal.target_pose.pose.position.y = -0.5;
  // set goal orientation
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;
  client.sendGoal(goal);
  client.waitForResult();
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("I am at the table!!");
  }
  */

  return 0;
}