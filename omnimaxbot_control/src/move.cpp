#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher goal_pub;

std_msgs::Bool goalReached;

double goalX;
double goalY;
double goalZ;
double goalW;

/*void move_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  goalReached.data = false;
  goalX = msg->position.x;
  goalY = msg->position.y;
  goalZ = msg->orientation.z;
  goalW = msg->orientation.w;
} */

//Takes an x position, y position, and orientation and sends it to move_base. These must be in the map frame
int move(double poseX, double poseY, double orientationZ, double orientationW)
{
  MoveBaseClient ac("move_base", true);
  move_base_msgs::MoveBaseGoal goal;

  //send the goal to the robot
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = poseX;
  goal.target_pose.pose.position.y = poseY;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = orientationZ;
  goal.target_pose.pose.orientation.w = orientationW;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("OmniMaxbot moved to position");
    goalReached.data = true;
    goal_pub.publish(goalReached);
  }
  else
  {
    ROS_INFO("The base failed to move");
  }

  return 0;
}

int main(int argc, char** argv)
{
  //node setup
  ros::init(argc, argv, "control");
  ros::NodeHandle n;

  //ros::Subscriber goal_sub = n.subscribe("move_goal", 1, move_callback);

  goal_pub = n.advertise<std_msgs::Bool>("goal_reached", 1);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  goalReached.data = false;

  goal_pub.publish(goalReached);

  move(3.4188326718,-0.393406369627,0.00252242960762,0.999996818669);

/*
  bool isFirst = true;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while(ros::ok())
  {
    if(isFirst)
    {
    }
    else if(!goalReached);
    {
      move(goalX, goalY, goalZ, goalW);
    }
  }

  spinner.stop();
*/
  return 0;
}
