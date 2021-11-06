#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    // initialize pick_objects node
    ros::init(argc, argv, "pick_objects");

    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // wait 5 seconds for move_base server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // define pick goal for the robot
    goal.target_pose.pose.position.x = -4.0;
    goal.target_pose.pose.position.y = -2.5;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending pick goal");
    ac.sendGoal(goal);

    // wait an infinite time for the results
    ac.waitForResult();

    // check if robot reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Reached pickup zone in map frame.");
    }
    else
    {
        ROS_INFO("Failed to reach pickup zone.");
    }

    // wait for 5 seconds after reaching pick goal
    ros::Duration(5).sleep();

    // define drop off goal
    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.position.y = -3.0;

    ROS_INFO("Sending drop off goal");
    ac.sendGoal(goal);

    // wait an infinite time for the results
    ac.waitForResult();

    // check if robot reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Reached drop off zone in map frame.");
    }
    else
    {
        ROS_INFO("Failed to reach drop off zone.");
    }

    // wait for 5 seconds after reaching drop off
    ros::Duration(5).sleep();

    return 0;
}