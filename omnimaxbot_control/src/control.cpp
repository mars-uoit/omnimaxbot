#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float x_dist;
float z_dist;

void arsys_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  x_dist = msg->vector.x;
  z_dist = msg->vector.z;
}

int main(int argc, char** argv)
{
  //node setup
  ros::init(argc, argv, "control");
  ros::NodeHandle n;
  
  //setup subscriber
  ros::Subscriber sub = n.subscribe("/ar_single_board/position", 1, arsys_callback);
  
  //publish the initial position
  ros::Publisher initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 5);
  
  geometry_msgs::PoseWithCovarianceStamped initialpose_msg;
  initialpose_msg.header.stamp = ros::Time::now();
  initialpose_msg.header.frame_id = "/map";
  initialpose_msg.pose.pose.position.x = -0.110;
  initialpose_msg.pose.pose.position.y = 0.336;
  initialpose_msg.pose.pose.position.z = 0.0;
  initialpose_msg.pose.pose.orientation.x = 0.0;
  initialpose_msg.pose.pose.orientation.y = 0.0;
  initialpose_msg.pose.pose.orientation.z = 0.007;
  initialpose_msg.pose.pose.orientation.w = 0.999297396179;
  //initialpose_msg.pose.covariance[] = [0.0024607411540919126, -0.0008915057573047717, 0.0, 0.0, 0.0, 0.0, -0.0008915057573047717, 0.006334715136075211, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.004074091031678167];
  initialpose_pub.publish(initialpose_msg);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
	  ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 2.891;
  goal.target_pose.pose.position.y = -0.408;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.025;
  goal.target_pose.pose.orientation.w = 0.999222750024;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  ROS_INFO("OmniMaxbot moved to first position");
  else
	  ROS_INFO("The base failed to move to the first position");
  
  ros::spin();
  
  //we'll send a goal to the robot
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x_dist - 0.055; //0.055 is the distance from the center of the camera to the right lens
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 0.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  ROS_INFO("OmniMaxbot moved to x position");
  else
	  ROS_INFO("The base failed to move to x position");
  
  ros::spin();
  
  //we'll send a goal to the robot
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 0.0 - (z_dist - 0.18);
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 0.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  ROS_INFO("OmniMaxbot moved to final position");
  else
	  ROS_INFO("The base failed to move to final");
  
  return 0;
}
