#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include "tf/LinearMath/Transform.h"

int main(int argc, char** argv)
{
  //node setup
  ros::init(argc, argv, "get_angles");

  double yaw = tf::getYaw(tf::Quaternion(0.716706982961,-0.69341563017,0.0584649248198,0.0456914696176));

  ROS_INFO_STREAM("Yaw: " << yaw << " radians");

  return 0;
}
