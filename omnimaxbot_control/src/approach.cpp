#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "phidgets/encoder_params.h"

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
bool moveGoalReached = false;
int frontForkPosition = 0;
int rearForkPosition = 0;

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
  double zGroundHeight = 0.1;//find this experimentally

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

void arsys_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  //converting from ar_sys frame to robot frame
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

void move_callback(const std_msgs::Bool::ConstPtr& msg)
{
  moveGoalReached = msg->data;
}

int main(int argc, char** argv)
{
  ROS_INFO("Made it into main");
  //node setup
  ros::init(argc, argv, "arsys_test");
  ros::NodeHandle n;
  
  //setup subscribers
  ros::Subscriber ar_sub = n.subscribe("/ar_single_board/position", 1, arsys_callback);
  ros::Subscriber front_fork_height_sub = n.subscribe("phidgets/motorcontrol/299103/encoder", 1, front_fork_height_callback);
  ros::Subscriber rear_fork_height_sub = n.subscribe("phidgets/motorcontrol/299104/encoder", 1, rear_fork_height_callback);
  ros::Subscriber front_fork_goal_sub = n.subscribe("phidgets/motorcontrol/299103/goal_reached", 1, front_fork_callback);
  ros::Subscriber rear_fork_goal_sub = n.subscribe("phidgets/motorcontrol/299104/goal_reached", 1, rear_fork_callback);
  ros::Subscriber move_goal_sub = n.subscribe("move_base_goal_reached", 1, move_callback);

  //setup fork goal publisher
  fork_pub = n.advertise<std_msgs::Float32>("fork_position", 1);

  //setup velocity publisher
  vel_pub = n.advertise<geometry_msgs::Twist>("omni_cmd_vel", 1);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while(ros::ok())
  {
    if(moveGoalReached)
    {
      line_up_x();

      approach(0.25); //found experimentally

      lift(0.0508); //lift 2 inches

      ros::Duration(2.0).sleep();  //wait 2 seconds

      lift(-0.0635); //put back down

      approach(0.90); //reverse enough for the forks to clear the can

      lift(0.0127); 
    }
  }

  spinner.stop();
    
  return 0;
}
