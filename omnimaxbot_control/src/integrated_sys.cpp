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

MoveBaseClient ac("move_base", true);

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
int move(double xGoal, double yGoal, double thetaGoal)
{
  move_base_msgs::MoveBaseGoal goal;

  //send the goal to the robot
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = xGoal;
  goal.target_pose.pose.position.y = yGoal;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(thetaGoal);

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
  ros::init(argc, argv, "control");
  ros::NodeHandle n;
  
  //setup subscriber
  ros::Subscriber ar_sub = n.subscribe("/ar_single_board/position", 1, arsys_callback);

  //setup subscriber
  ros::Subscriber goal_sub = n.subscribe("fork_goal_reached", 1, fork_callback);

  //setup fork goal publisher
  ros::Publisher fork_pub = n.advertise<std_msgs::Float32>("fork_position", 1);

  //setup velocity publisher
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("omni_cmd_vel", 1);

  //tell the action client that we want to spin a thread by default
  //MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  //move to pick up position (position found through teleoperation and amcl)
  move(3.2904797043,-0.506466412138,0.0);

  line_up_x();

  approach(0.25); //found experimentally

  lift(0.0508); //lift 2 inches

  move(1,1,3.14159/2); //move to drop off location FIGURE THIS OUT EXPERIMENTALLY

  lift(-0.0508); //put back down

  approach(-0.20); //reverse enough for the forks to clear the can

  move(0,0,0); //move back to home
    
  return 0;
}
