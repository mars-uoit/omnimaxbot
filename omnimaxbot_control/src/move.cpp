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

bool pickup = false;
bool dropoff = false;

double goalX;
double goalY;
double goalZ;
double goalW;

//Takes an x position, y position, and orientation and sends it to move_base. These must be in the map frame
int move(double poseX, double poseY, double orientationZ, double orientationW)
{
  ROS_INFO("In move()");
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
    //goalReached.data = true;
    //goal_pub.publish(goalReached);
  }
  else
  {
    ROS_INFO("The base failed to move");
  }

  return 0;
}

void pickup_callback(const std_msgs::Bool::ConstPtr& msg)
{
  pickup = msg->data;
}

void dropoff_callback(const std_msgs::Bool::ConstPtr& msg)
{
  dropoff = msg->data;
}

int main(int argc, char** argv)
{
  //node setup
  ros::init(argc, argv, "move");
  ros::NodeHandle n;

  ros::Subscriber pickup = n.subscribe("pickup_reached", 1, pickup_callback);
  ros::Subscriber dropoff = n.subscribe("dropoff_reached", 1, dropoff_callback);

  ros::Publisher goal1_pub = n.advertise<std_msgs::Bool>("goal1_reached", 1);
  ros::Publisher goal2_pub = n.advertise<std_msgs::Bool>("goal2_reached", 1);

  bool done = false;
  bool firstMove = false;
  bool secondMove = false;

  std_msgs::Bool goal1Reached;
  std_msgs::Bool goal2Reached;

  goal1Reached.data = false;
  goal2Reached.data = false;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("Moving to pick-up location");


  goal1_pub.publish(goal1Reached);
  goal2_pub.publish(goal2Reached);

  move(3.4188326718,-0.443406369627,0.00252242960762,0.999996818669);

  goal1Reached.data = true;
  goal2Reached.data = false;
  goal1_pub.publish(goal1Reached);
  goal2_pub.publish(goal2Reached);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while(ros::ok() && done == false)
  {
    if(pickup && !firstMove)
    {
      ROS_INFO("Moving to drop-off location");

      move(2.15884737476,-0.322894082763,0.999994485685,0.00332093362971);

      goal1Reached.data = false;
      goal2Reached.data = true;
      goal1_pub.publish(goal1Reached);
      goal2_pub.publish(goal2Reached);

      firstMove = true;
    }
    
    if(dropoff && !secondMove)
    {
      ROS_INFO("Moving to stop location");

      move(0.00301826748789,-0.156723602908,0.00796528261361,0.999968276633);

      goal1Reached.data = false;
      goal2Reached.data = false;
      goal1_pub.publish(goal1Reached);
      goal2_pub.publish(goal2Reached);

      done = true;
      secondMove = true;
    }
  goal1_pub.publish(goal1Reached);
  goal2_pub.publish(goal2Reached);
  }

  ROS_INFO("exiting move.cpp");

  spinner.stop();

  return 0;
}
