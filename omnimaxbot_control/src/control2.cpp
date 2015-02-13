#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher initialpose_pub;
MoveBaseClient ac;

float x_dist;
float z_dist;

int move()
{
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 3.9596981839;
  goal.target_pose.pose.position.y = -0.532147080529;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.00763461178259;
  goal.target_pose.pose.orientation.w = 0.999970855927;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  ROS_INFO("OmniMaxbot moved to position");
  else
	  ROS_INFO("The base failed to move");

  return 0;
} 

int approach_x()
{
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.0 - x_dist;
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
	  ROS_INFO("OmniMaxbot moved to position");
  else
	  ROS_INFO("The base failed to move");

  return 0;
} 

int approach_y()
{
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 0.0 - (z_dist - 0.23);
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 0.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  ROS_INFO("OmniMaxbot moved to position");
  else
	  ROS_INFO("The base failed to move");

  return 0;
} 

void arsys_callback(const geometry_msgs::Vecto3Stamped::ConstPtr& msg)
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
  initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initalpose", 5);
  
  geometry_msgs::PoseWithCovarianceStamped initialpose_msg;
  initialpose_msg.header.stamp = ros::Time::now();
  initialpose_msg.header.frame_id = "/map";
  initialpose_msg.pose.pose.position.x = 0.00081265325688;
  initialpose_msg.pose.pose.position.y = 0.302899883754;
  initialpose_msg.pose.pose.position.z = 0.0;
  initialpose_msg.pose.pose.orientation.x = 0.0;
  initialpose_msg.pose.pose.orientation.y = 0.0;
  initialpose_msg.pose.pose.orientation.z = -0.0031009447888;
  initialpose_msg.pose.pose.orientation.w = 0.999995192059;
  initialpose_msg.covariance = [0.23794657230693583, -0.0051372335288317975, 0.0, 0.0, 0.0, 0.0, -0.005137233528831803, 0.21520926795279804, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.058171250299444636];
  initailpose_pub.publish(initalpose_msg);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
	  ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  move();
  
  ros::SpinOnce();
  
  approach_x();
  
  ros::spinOnce();
  
  approach_y();
  
  return 0;
}
