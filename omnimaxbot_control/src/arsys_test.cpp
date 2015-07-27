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

//lines the forks up with the can
int line_up_x()
{
  bool isLinedUp = false;
  double goal = 0.105; //found experimentally
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
    error = goal - xDist;

    if ((error > 0 && error <= tolerance) || (error < 0 && error > -tolerance))
    {
      isLinedUp = true;
      vel.linear.x = 0.0;
    }
    else if (error < 0)
    {
      vel.linear.x = 0.25;
    }
    else
    {
      vel.linear.x = 0.25;
    }
    vel_pub.publish(vel); 
  }

  spinner.stop();

  return 0;
} 

int approach(double goal)
{
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
    else if (error > 0)
    {
      vel.linear.y = 0.25;
    }
    else
    {
      vel.linear.y = 0.25;
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
  double offset = 0.254;
  double startHeight = 0.250825;

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
  xDist = msg->vector.y;
  yDist = msg->vector.z;
  zDist = msg->vector.x;
}

void fork_callback(const std_msgs::Bool::ConstPtr& msg)
{
  metGoal = msg->data;
}


int main(int argc, char** argv)
{
  //node setup
  ros::init(argc, argv, "arsys_test");
  ros::NodeHandle n;
  
  //setup subscriber
  ros::Subscriber ar_sub = n.subscribe("/ar_single_board/position", 1, arsys_callback);

  //setup subscriber
  ros::Subscriber goal_sub = n.subscribe("fork_goal_reached", 1, fork_callback);

  //setup fork goal publisher
  ros::Publisher fork_pub = n.advertise<std_msgs::Float32>("fork_position", 1);

  //setup velocity publisher
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("omni_cmd_vel", 1);

  line_up_x();

  approach(0.25); //found experimentally

  lift(0.0508); //lift 2 inches

  lift(-0.0508); //put back down

  approach(0.90); //reverse enough for the forks to clear the can
    
  return 0;
}
