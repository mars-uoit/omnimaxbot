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
#include "phidgets/encoder_params.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//fork height publisher
ros::Publisher fork_pub;

//command velocity publisher
ros::Publisher vel_pub;

//global variables to hold can position
float xDist;
float yDist;
float zDist;

//global variables for callback
bool frontForkGoalReached = false;
bool rearForkGoalReached = false;
int frontForkPosition = 0;
int rearForkPosition = 0;

struct poses
{
  double x;
  double y;
  double z;
  double w;
}pickup1, pickup2, dropoff, init;

void set_poses()
{
  pickup1.x = 3.4188326718;
  pickup1.y = -0.443406369627;
  pickup1.z = 0.00252242960762;
  pickup1.w = 0.999996818669;

  pickup2.x = 3.4188326718;
  pickup2.y = -0.443406369627;
  pickup2.z = 0.00252242960762;
  pickup2.w = 0.999996818669;

  dropoff.x = 3.4188326718;
  dropoff.y = -0.443406369627;
  dropoff.z = 0.00252242960762;
  dropoff.w = 0.999996818669;

  init.x = 0.0;
  init.y = 0.0;
  init.z = 0.0;
  init.w = 0.0;
}

//Takes an x position, y position, and orientation and sends it to move_base. These must be in the map frame
int move(double poseX, double poseY, double orientationZ, double orientationW)
{
  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

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
  }
  else
  {
    ROS_INFO("The base failed to move");
  }

  ac.cancelAllGoals();

  return 0;
}

//lines the forks up with the can
int line_up_x()
{
  bool isLinedUp = false;
  double goal = 0.0472; //found experimentally
  double error;
  double tolerance = 0.0381; // +/-1.5 inches

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

    if ((error > 0 && error <= tolerance) || (error < 0 && error > -tolerance))
    {
      isLinedUp = true;
      vel.linear.x = 0.0;
    }
    else if (error < 0)
    {
      vel.linear.x = -0.05;
    }
    else
    {
      vel.linear.x = 0.05;
    }
    vel_pub.publish(vel); 
  }

  spinner.stop();

  return 0;
} 

int approach(double goalY)
{
  bool isClose = false;
  bool xTol = false;
  double errorX;
  double errorY;
  double toleranceY = 0.025;
  double toleranceX = 0.0381;
  double goalX = 0.0472;

  geometry_msgs::Twist vel;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok() && isClose == false)
  {
    errorY = goalY - yDist;
    errorX = xDist - goalX;

    if ((errorX > 0 && errorX <= toleranceX) || (errorX < 0 && errorX > -toleranceX))
    {
      vel.linear.x = 0.0;
      xTol = true;
    }
    else if (errorX < 0)
    {
      vel.linear.x = -0.05;
      xTol = false;
    }
    else
    {
      vel.linear.x = 0.05;
      xTol = false;
    }
      
    if ((errorY > 0 && errorY <= toleranceY) || (errorY < 0 && errorY > -toleranceY))
    {
      isClose = true;
      vel.linear.y = 0.0;
    }
    else if (errorY < 0 && xTol == true)
    {
      vel.linear.y = -0.1;
    }
    else if (errorY > 0 && xTol == true)
    {
      vel.linear.y = 0.1;
    }
    vel_pub.publish(vel); 
  }

  spinner.stop();

  return 0;
}

void front_fork_height_callback(const phidgets::encoder_params::ConstPtr& msg)
{
    frontForkPosition = msg->count;
}

void rear_fork_height_callback(const phidgets::encoder_params::ConstPtr& msg)
{
    rearForkPosition = msg->count;
}

//tell forks to raise/lower by a certain amount
int lift(double dist)
{
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  double conv = 39.3700787; //inchs per metre
  double gearRatio = 13.0;
  double TPI = 10.0; //threads per inch
  double cpr = 300.0; //from encoder
  double frFrkPs = frontForkPosition;
  double rrFrkPs = rearForkPosition;
  double avgPs = (((((frFrkPs + rrFrkPs)/2)/cpr)/gearRatio)/TPI)/conv; //find the average encoder count position and convert from encoder counts to metres
  std_msgs::Float32 goal;
  goal.data = avgPs + dist;
  
  fork_pub.publish(goal);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok() && frontForkGoalReached == false && rearForkGoalReached == false)
  {
    fork_pub.publish(goal);
  }

  spinner.stop();

  return 0;
}

//tell forks to lower the can
//this is found by knowing what zDist is when the can is on the ground
int drop()
{
  ros::Duration(0.5).sleep();
  double offset = 0.297;
  double zGroundHeight = 0.1;//FIND THIS

  std_msgs::Float32 goal;
  goal.data = zGroundHeight - zDist - 0.0127; //lower 0.5 inch lower than needed to ensure that the can is off of the forks 

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok() && frontForkGoalReached == false && rearForkGoalReached == false)
  {
    fork_pub.publish(goal);
  }

  spinner.stop();

  return 0;
}

//returns forks to their 0 position (pickup height)
int prep()
{
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  double conv = 39.3700787; //inchs per metre
  double gearRatio = 13.0;
  double TPI = 10.0; //threads per inch
  double cpr = 300.0; //from encoder
  double frFrkPs = frontForkPosition;
  double rrFrkPs = rearForkPosition;
  double avgPs = (((((frFrkPs + rrFrkPs)/2)/cpr)/gearRatio)/TPI)/conv; //find the average encoder count position and convert from encoder counts to metres
  
  if(avgPs == 0)
  {
    //do nothing
  }
  else
  {
    std_msgs::Float32 goal;
    goal.data = avgPs * -1;
  
    fork_pub.publish(goal);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (ros::ok() && frontForkGoalReached == false && rearForkGoalReached == false)
    {
      fork_pub.publish(goal);
    }

    spinner.stop();
  }

  return 0;
}

void arsys_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  //convert from ar_sys frame to robot frame
  xDist = msg->vector.x * -1;
  yDist = msg->vector.z;
  zDist = msg->vector.y * -1;
}

void front_fork_callback(const std_msgs::Bool::ConstPtr& msg)
{
  frontForkGoalReached = msg->data;
}

void rear_fork_callback(const std_msgs::Bool::ConstPtr& msg)
{
  rearForkGoalReached = msg->data;
}

int main(int argc, char** argv)
{
  //node setup
  ros::init(argc, argv, "integrated_sys");
  ros::NodeHandle n;
  
  //setup subscribers
  ros::Subscriber ar_sub = n.subscribe("/ar_single_board/position", 1, arsys_callback);
  ros::Subscriber front_fork_height_sub = n.subscribe("phidgets/motorcontrol/299103/encoder", 1, front_fork_height_callback);
  ros::Subscriber rear_fork_height_sub = n.subscribe("phidgets/motorcontrol/299104/encoder", 1, rear_fork_height_callback);
  ros::Subscriber front_fork_goal_sub = n.subscribe("phidgets/motorcontrol/299103/goal_reached", 1, front_fork_callback);
  ros::Subscriber rear_fork_goal_sub = n.subscribe("phidgets/motorcontrol/299104/goal_reached", 1, rear_fork_callback);

  //setup fork goal publisher
  fork_pub = n.advertise<std_msgs::Float32>("fork_position", 1);

  //setup velocity publisher
  vel_pub = n.advertise<geometry_msgs::Twist>("omni_cmd_vel", 1);

  set_poses();

  move(pickup1.x, pickup1.y, pickup1.z, pickup1.w);

  prep();

  line_up_x();

  approach(0.25); //found experimentally

  lift(0.0508); //lift 2 inches

  approach(0.90);

  move(dropoff.x, dropoff.y, dropoff.z, dropoff.w);

  drop();

  approach(0.90);

  move(init.x, init.y, init.z, init.w);

  prep();

  return 0;
}
