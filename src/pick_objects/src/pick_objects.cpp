#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client to send goal requests to move_base server through SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 4.7;
  goal.target_pose.pose.position.y = 3.5;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick-up goal...");
  ac.sendGoal(goal);
  ROS_INFO("Navigating towards pick-up location...");

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("<<---Robot reached pick-up--->>");
    // Wait for 5 seconds
    ros::Duration(5.0).sleep();
    
    // Define a second goal
    goal.target_pose.pose.position.x = 2.8;
    goal.target_pose.pose.position.y = -0.1;
    goal.target_pose.pose.orientation.w = -1.0;
    
    ROS_INFO("Sending drop-off goal...");
    ac.sendGoal(goal);
    ROS_INFO("Navigating towards drop-off location...");

    // Wait an infinite time for the results
    ac.waitForResult();  
    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("<<---Robot reached drop-off--->>");
    else
      ROS_INFO("Robot failed to reach drop-off goal.");
    
    ros::Duration(5.0).sleep();
  }
  else
    ROS_INFO("Robot failed to reach pick-up goal.");
  return 0;
}