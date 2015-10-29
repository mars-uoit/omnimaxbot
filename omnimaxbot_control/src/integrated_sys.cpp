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
float canXDist;
float canYDist;
float canZDist;
double canTheta;

//global variables to hold drop position
float dropXDist;
float dropYDist;
double dropTheta;

//global variables for callback
bool frontForkGoalReached = false;
bool rearForkGoalReached = false;
int frontForkPosition = 0;
int rearForkPosition = 0;

//variables for raising/lowering forks
double conv = 39.3700787; //inchs per metre
double gearRatio = 13.0;
double TPI = 10.0; //threads per inch
double cpr = 300.0; //from encoder

struct poses
{
  double x;
  double y;
  double z;
  double w;
}pickup1, dropoff, init;

void set_poses()
{
  pickup1.x = 4.06052467576;
  pickup1.y = 1.43293669096;
  pickup1.z = -0.00870027222147;
  pickup1.w = 0.999962151915;

  dropoff.x = 2.23452364634;
  dropoff.y = 1.53506418211;
  dropoff.z = -0.999963292419;
  dropoff.w = 0.00856818618366;

  init.x = 0.0529914905167;
  init.y = 1.15482337803;
  init.z = -0.0102563308521;
  init.w = 0.999947402455;
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

  return 0;
}

//lines the forks up with the can
int line_up_x()
{
  bool isLinedUp = false;
  bool isAngled = true;
  bool isFirst = true;
  double xGoal = 0.0472; //found experimentally
  double xError;
  double xTolerance = 0.0381; // +/-1.5 inches
  double thGoal = 1.570796327; //pi/2 found experimentally
  double thErrorNow = 0;
  double thErrorLast = 0;
  double thTolerance = 0.0872664626; // +/- 5 deg (need to test this)
 
  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok() && isLinedUp == false)
  {
    xError = canXDist - xGoal;

    if ((xError >= 0 && xError <= xTolerance) || (xError < 0 && xError > -xTolerance))
    {
      isLinedUp = true;
      vel.linear.x = 0.0;
    }
    else if (xError < 0)
    {
      vel.linear.x = -0.05;
    }
    else if (xError > 0)
    {
      vel.linear.x = 0.05;
    }
    vel_pub.publish(vel); 
  }

  vel.linear.x = 0.0;

  while (ros::ok() && isAngled == true)
  {
    thErrorNow = thGoal - canTheta;

    if(thErrorNow <= thTolerance)
    {
      isAngled = false;
      vel.angular.z = 0.0;
    }
    else if(thErrorLast > thErrorNow)
    {
      vel.angular.z = 0.1;
    }
    else
    {
      vel.angular.z = -0.1;
    }
    vel_pub.publish(vel);
    thErrorLast = thErrorNow;
  }

  spinner.stop();

  return 0;
} 

int approach_can(double goalY)
{
  bool isClose = false;
  bool xTol = false;
  double errorX;
  double errorY;
  double toleranceY = 0.025;
  double toleranceX = 0.0381;
  double goalX = 0.0472;

  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok() && isClose == false)
  {
    errorY = goalY - canYDist;
    errorX = canXDist - goalX;

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

  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  
  vel_pub.publish(vel);

  spinner.stop();

  return 0;
}

int approach_drop()
{
  bool isClose = false;
  bool xTol = false;
  bool isAngled = true;
  double errorX;
  double errorY;
  double errorTh;
  double toleranceX = 0.0127;
  double toleranceY = 0.025;
  double toleranceTh = 0.0872664626; // +/- 5 deg (need to test this)
  double goalX = 0.0206; //0.0356;
  double goalY = 1.7915; //1.4;
  double goalTh = 1.570796327; //pi/2 found experimentally
  double errorThNow = 0;
  double errorThLast = 0;

  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok() && isClose == false)
  {
    errorY = goalY - dropYDist;
    errorX = dropXDist - goalX;
    errorThNow = goalTh - dropTheta;

    while(isAngled == true)
    {
      errorThNow = goalTh - dropTheta;
      if(errorThNow <= toleranceTh)
      {
        isAngled = false;
        vel.angular.z = 0.0;
      }
      else if(errorThLast > errorThNow)
      {
        vel.angular.z = 0.1;
      }
      else
      {
        vel.angular.z = -0.1;
      }
      vel_pub.publish(vel);
      errorThLast = errorThNow;
    }

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

  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.angular.z = 0.0;

  vel_pub.publish(vel);

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
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  double offset = 0.29845; 
  double preset = 0.30384301;
  double frFrkPs = frontForkPosition;
  double rrFrkPs = rearForkPosition;
  double avgPs = (((((frFrkPs + rrFrkPs)/2)/cpr)/gearRatio)/TPI)/conv; //find the average encoder count position and convert from encoder counts to metres
  bool forksMetGoal = false;

  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  std_msgs::Float32 goal;
  goal.data = canZDist + offset + dist - preset;
  //goal.data = dist;
  
  fork_pub.publish(goal);

  ros::Duration(0.5).sleep();

  ros::AsyncSpinner spinner(4);
  spinner.start();

  forksMetGoal = false;

  while (ros::ok() && forksMetGoal == false)
  {
    //ROS_INFO_STREAM("forksMetGoal start: " << forksMetGoal);
    if(frontForkGoalReached == true && rearForkGoalReached == true)
    {
      forksMetGoal = true;
    }
    else
    {
      fork_pub.publish(goal);
      forksMetGoal = false;
    }
    //ROS_INFO_STREAM("frontForkGoalReached: " << frontForkGoalReached);
    //ROS_INFO_STREAM("rearForkGoalReached: " << rearForkGoalReached);
    //ROS_INFO_STREAM("forksMetGoal end: " << forksMetGoal);
  }

  ros::Duration(0.5).sleep(); 

  spinner.stop();

  return 0;
}

int reverse()
{
  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.y = 0.1;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  int stop = 8;
  ros::Time start = ros::Time::now(); 

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while(ros::ok() && (ros::Time::now() - start).toSec() < stop)
  {
    vel_pub.publish(vel);
  }  

  spinner.stop();

  return 0; 
}

void lift_arsys_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //convert from ar_sys frame to robot frame
  canXDist = msg->pose.position.x * -1;
  canYDist = msg->pose.position.z;
  canZDist = msg->pose.position.y * -1;
  canTheta = tf::getYaw(msg->pose.orientation) * -1;
}

void drop_arsys_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //convert from ar_sys frame to robot frame
  dropXDist = msg->pose.position.x * -1;
  dropYDist = msg->pose.position.z;
  dropTheta = tf::getYaw(msg->pose.orientation) * -1;
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
  ros::Subscriber lift_ar_sub = n.subscribe("/can_ar_sys/pose", 1, lift_arsys_callback);
  ros::Subscriber drop_ar_sub = n.subscribe("/drop_ar_sys/pose", 1, drop_arsys_callback);
  ros::Subscriber front_fork_height_sub = n.subscribe("phidgets/motorcontrol/299103/encoder", 1, front_fork_height_callback);
  ros::Subscriber rear_fork_height_sub = n.subscribe("phidgets/motorcontrol/299104/encoder", 1, rear_fork_height_callback);
  ros::Subscriber front_fork_goal_sub = n.subscribe("phidgets/motorcontrol/299103/goal", 1, front_fork_callback);
  ros::Subscriber rear_fork_goal_sub = n.subscribe("phidgets/motorcontrol/299104/goal", 1, rear_fork_callback);

  //setup fork goal publisher
  fork_pub = n.advertise<std_msgs::Float32>("fork_position", 1);

  //setup velocity publisher
  vel_pub = n.advertise<geometry_msgs::Twist>("omni_cmd_vel", 1);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  set_poses();

  move(pickup1.x, pickup1.y, pickup1.z, pickup1.w);

  lift(0.0);

  line_up_x();

  approach_can(0.25); //found experimentally

  lift(0.0762); //lift 3 inches

  reverse();

  move(dropoff.x, dropoff.y, dropoff.z, dropoff.w);

  approach_drop();

  lift(0.0);

  approach_can(0.90);

  move(init.x, init.y, init.z, init.w);

  return 0;
}
