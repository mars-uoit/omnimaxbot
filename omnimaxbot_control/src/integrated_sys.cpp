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

  //send the goal to the robot (NEED TO CHECK LOCATIONS FOR NEW MAP)
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

  return 0;
} 

int approach_y()
{

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
