#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

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

bool goalReached = false;

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
    //ROS_INFO_STREAM("goal = " << goal);
    //ROS_INFO_STREAM("xDist = " << xDist);
    //ROS_INFO_STREAM("Error = " << error);

    if ((error > 0 && error <= tolerance) || (error < 0 && error > -tolerance))
    {
      isLinedUp = true;
      vel.linear.x = 0.001;
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

void goal_callback(const std_msgs::Bool::ConstPtr& msg)
{
  goalReached = msg->data;
}

int main(int argc, char** argv)
{
  ROS_INFO("Made it into main");
  //node setup
  ros::init(argc, argv, "arsys_test");
  ros::NodeHandle n;
  
  //setup subscriber
  ros::Subscriber ar_sub = n.subscribe("/ar_single_board/position", 1, arsys_callback);

  //setup subscriber
  ros::Subscriber goal_sub = n.subscribe("fork_goal_reached", 1, fork_callback);

  //setup subscriber
  ros::Subscriber move_goal_sub = n.subscribe("goal_reached", 1, goal_callback);

  //setup fork goal publisher
  fork_pub = n.advertise<std_msgs::Float32>("fork_position", 1);

  //setup velocity publisher
  vel_pub = n.advertise<geometry_msgs::Twist>("omni_cmd_vel", 1);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while(ros::ok())
  {
    if(goalReached)
    {
      line_up_x();

      approach(0.25); //found experimentally

      /*
        lift(0.0508); //lift 2 inches

        lift(-0.0508); //put back down

        approach(0.90); //reverse enough for the forks to clear the can
      */
    }
  }

  spinner.stop();
    
  return 0;
}
