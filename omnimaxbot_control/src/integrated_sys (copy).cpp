#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//fork height publisher
ros::Publisher fork_pub;

//command velocity publisher
ros::Publisher vel_pub;

//global variables to hold can position
float xDist;
float yDist;
float zDist;

//global variable for callback
bool metGoal = false;

//Takes an x position, y position, and orientation and sends it to move_base. These must be in the map frame
//int move(double poseX, double poseY, double angle)
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
  //goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = orientationZ;
  goal.target_pose.pose.orientation.w = orientationW;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("OmniMaxbot moved to position");
  else
    ROS_INFO("The base failed to move");

  return 0;
} 

//lines the forks up with the can
int line_up_x()
{
  ROS_INFO("Entered approach()");
  bool isLinedUp = false;
  double goal = 0.04; //found experimentally
  double error;
  double tolerance = 0.0025;

  geometry_msgs::Twist vel;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok() && isLinedUp == false)
  {
    error = xDist - goal;
    ROS_INFO_STREAM("goal = " << goal);
    ROS_INFO_STREAM("xDist = " << xDist);
    ROS_INFO_STREAM("Error = " << error);

    if ((error > 0 && error <= tolerance) || (error < 0 && error > -tolerance))
    {
      isLinedUp = true;
      vel.linear.x = 0.0;
    }
    else if (error < 0)
    {
      vel.linear.x = -0.10;
    }
    else
    {
      vel.linear.x = 0.10;
    }
    ROS_INFO_STREAM("vel.linear.x = " << vel.linear.x);
    vel_pub.publish(vel); 
  }

  spinner.stop();

  return 0;
} 

int approach(double goal)
{
  ROS_INFO("Entered approach()");
  bool isClose = false;
  double error;
  double tolerance = 0.025;

  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok() && isClose == false)
  {
    error = goal - yDist;

    if ((error > 0 && error <= tolerance) || (error < 0 && error > -tolerance))
    {
      isClose = true;
      vel.linear.y = 0.0;
    }
    else if (error < 0)
    {
      vel.linear.y = -0.1;
    }
    else
    {
      vel.linear.y = 0.1;
    }
    vel_pub.publish(vel);
  }

  spinner.stop();

  ROS_INFO_STREAM("OmniMaxbot " << goal << " m from can");

  return 0;
}
 
//tell forks to raise/lower by a certain amount, knowing the distance between the AR code and the flange
int lift(double dist)
{
  bool isFirst = true;
  double offset = 0.346;
  double startHeight = 0.2725;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok() && metGoal == false)
  {
    if (isFirst)
    {
      ros::spinOnce();
      std_msgs::Float32 goal;
      goal.data = zDist + offset + dist - startHeight;
      fork_pub.publish(goal);
    }
  }

  spinner.stop();

  ROS_INFO("Forks at desired height");

  return 0;
}

void arsys_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  //converting from ar_sys frame to robot frame
  xDist = msg->vector.x * -1;
  yDist = msg->vector.z;
  zDist = msg->vector.y * -1;
}

void fork_callback(const std_msgs::Bool::ConstPtr& msg)
{
  metGoal = msg->data;
}


int main(int argc, char** argv)
{
  //node setup
  ros::init(argc, argv, "control");
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  //setup subscriber
  ros::Subscriber ar_sub = n.subscribe("/ar_single_board/position", 1, arsys_callback);

  //setup subscriber
  ros::Subscriber goal_sub = n.subscribe("fork_goal_reached", 1, fork_callback);

  //setup fork goal publisher
  ros::Publisher fork_pub = n.advertise<std_msgs::Float32>("fork_position", 1);

  //setup velocity publisher
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("omni_cmd_vel", 1);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move(3.4188326718,-0.393406369627,0.00252242960762,0.999996818669);

  line_up_x();

  ROS_INFO("Exited line_up_x()");

  approach(0.25); //found experimentally

  ROS_INFO("Exited approach()");
  /*
  lift(0.0508); //lift 2 inches

  move(1,1,3.14159/2); //move to drop off location FIGURE THIS OUT EXPERIMENTALLY

  lift(-0.0508); //put back down

  approach(-0.20); //reverse enough for the forks to clear the can

  move(0,0,0); //move back to home
  */  
  return 0;
}
