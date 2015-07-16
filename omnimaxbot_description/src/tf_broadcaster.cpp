#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  
  double pi = 3.1415926535897932385;

  while(n.ok())
  {
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"base_footprint", "base_link"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.4826, 0.5461, 0.125)),
        ros::Time::now(),"base_footprint", "wheel_1"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.4826, 0.5461, 0.125)),
        ros::Time::now(),"base_footprint", "wheel_2"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.4826, -0.5461, 0.125)),
        ros::Time::now(),"base_footprint", "wheel_3"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.4826, -0.5461, 0.125)),
        ros::Time::now(),"base_footprint", "wheel_4"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.6858, 0.0, 0.6604)),
        ros::Time::now(),"base_link", "front_laser"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, pi)), tf::Vector3(-0.6858, 0.0, 0.6604)),
        ros::Time::now(),"base_link", "rear_laser"));
    
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, -pi/2)), tf::Vector3(0.0, -0.08255, 1.2446)),
        ros::Time::now(),"base_link", "kinect"));
        
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, 0.0)), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"kinect", "camera_link"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, -pi/2)), tf::Vector3(0.0, 0.0375, 0.44)),
        ros::Time::now(),"base_link", "stereo_camera"));
        
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.013, 0.504, 0.465)),
        ros::Time::now(),"base_link", "imu_link"));

    r.sleep();
  }
}
