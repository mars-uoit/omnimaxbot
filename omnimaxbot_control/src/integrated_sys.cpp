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

int main(int argc, char** argv)
{
  //node setup
  ros::init(argc, argv, "control");
  ros::NodeHandle n;

  //Publisher setup
  ros::Publisher goal_pub = n.advertise<geometry_msgs::Pose>("move_goal", 1);
  
  geometry_msgs::Pose pose;

  //send the goal
  pose.position.x = 3.4188326718;
  pose.position.y = -0.393406369627;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.00252242960762;
  pose.orientation.w = 0.999996818669;
  goal_pub.publish(pose);

  return 0;
}
