#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include "tf/LinearMath/Transform.h"

int main(int argc, char** argv)
{
  //node setup
  ros::init(argc, argv, "get_angles");

  double pi = 3.141592653589793;

  //double yaw_straight_rad = tf::getYaw(tf::Quaternion(0.716706982961,-0.69341563017,0.0584649248198,0.0456914696176)) * -1;
  //double yaw_can_clock_rad = tf::getYaw(tf::Quaternion(0.70459528068,-0.703025019294,-0.0426174293426,0.0865162182984)) * -1;
  //double yaw_can_counter_rad = tf::getYaw(tf::Quaternion(0.703988117207,-0.69213448887,0.122547744425,-0.101649482797)) * -1;
  double yaw_static_obs_rad = tf::getYaw(tf::Quaternion(0.0,0.0,-0.999879200489,0.0155429865115)) * -1;

  //double yaw_straight_deg = yaw_straight_rad * 180/pi;
  //double yaw_can_clock_deg = yaw_can_clock_rad * 180/pi;
  //double yaw_can_counter_deg = yaw_can_counter_rad * 180/pi;

  //ROS_INFO_STREAM("Yaw straight on: " << yaw_straight_rad << " radians or " << yaw_straight_deg << " degrees");
  //ROS_INFO_STREAM("Yaw with can rotated clockwise: " << yaw_can_clock_rad << " radians or " << yaw_can_clock_deg << " degrees");
  //ROS_INFO_STREAM("Yaw with can rotated counter-clockwise: " << yaw_can_counter_rad << " radians or " << yaw_can_counter_deg << " degrees");
  ROS_INFO_STREAM("Initial_pose_a: " << yaw_static_obs_rad);

  return 0;
}
