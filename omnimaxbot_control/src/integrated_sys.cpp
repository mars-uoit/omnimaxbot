#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient ac;

//fork height publisher
ros::Publisher fork_pub;

//command velocity publisher
ros::Publisher cmd_vel;

//global variables to hold can position
float x_dist;
float y_dist;
float z_dist;

//global variable for callback
bool metGoal = false;

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

  ros::

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

//tell forks to raise/lower by a certain amount, knowing the distance between the AR code and the flange
int lift(double dist)
{
  bool isFirst = true;
  double 

  ros::Asyncspinner spinner(4);
  spinner.start();

  while(ros::ok() && metGoal == false)
  {
    if(isFirst)
    {
      std_msgs::Float32 goal.data = zDist + offset + dist - startHeight;   
    }  
  {

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

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  //move to pick up position (position found through teleoperation and amcl)
  move(3.2904797043,-0.506466412138,0.0)

  line_up_x();

  approach()

  lift(0.0508) //lift 2 inches

  move(x,y,t) //move to drop off location FIGURE THIS OUT EXPERIMENTALLY

  lift(-0.0508) //put back down

  approach() //the other way

  move(x,y,t) //move back to home
    
  return 0;
}
