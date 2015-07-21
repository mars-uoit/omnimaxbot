#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient ac;

float x_dist;
float y_dist;
float z_dist;

//Takes an x position, y position, and orientation and sends it to move_base. These must be in the map frame
int move(double xGoal, double yGoal, double thetaGoal)
{
  move_base_msgs::MoveBaseGoal goal;

  //send the goal to the robot (NEED TO CHECK LOCATIONS FOR NEW MAP)
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = xGoal;
  goal.target_pose.pose.position.y = yGoal;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation = tf::createQuaternionFromRPY(0.0, 0.0, thetaGoal)

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("OmniMaxbot moved to position");
  else
    ROS_INFO("The base failed to move");

  return 0;
} 

\\lines the forks up with the can
int line_up_x()
{
  bool isLinedUp = false;

  while (isLinedUp == false)
  {
    double error = 
  }

  return 0;
} 

int approach()
{

  return 0;
} 

int lift()
{

  return 0;
}

void arsys_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  x_dist = msg->vector.x;
  y_dist = msg->vector.y;
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
  

  
  return 0;
}
