#include <exception>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include "robotic_pusher/moveTiago.h"

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JointClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveClient;

bool move_to_des_pose(robotic_pusher::moveTiago::Request  &req, robotic_pusher::moveTiago::Response &res){
    /****************************************************************/
    /***********************Move to table****************************/
    /****************************************************************/
    ROS_INFO("Starting moving to table");
    MoveClient client("move_base", true); // true -> don't need ros::spin()
    client.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    // reference frame is map
    goal.target_pose.header.frame_id = "map";
    // set goal as 2D coordinate
    goal.target_pose.pose.position.x = req.desPose.position.x;
    goal.target_pose.pose.position.y = req.desPose.position.y;
    // set goal orientation
    goal.target_pose.pose.orientation.x = req.desPose.orientation.x;
    goal.target_pose.pose.orientation.y = req.desPose.orientation.y;
    goal.target_pose.pose.orientation.z = req.desPose.orientation.z;
    goal.target_pose.pose.orientation.w = req.desPose.orientation.w;
    client.sendGoal(goal);
    client.waitForResult();
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("I am achieved the desired Pose!!");
    }
    else {
        ROS_ERROR("Failed to move to the desired Pose :(");
        return false;
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "intialize_tiago");
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

    ros::ServiceServer service = nh.advertiseService("moveTiago_service", move_to_des_pose);

    return 0;
}