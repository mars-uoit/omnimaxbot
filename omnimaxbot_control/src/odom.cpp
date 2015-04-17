#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>

double x_new = 0;
double y_new = 0;
double th_new = 0;
ros::Time time_new;

//function to save the new pose
void PoseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  time_new = msg->header.stamp;
  x_new = msg->pose.position.x;
  y_new = msg->pose.position.y;
  th_new = tf::getYaw(msg->pose.orientation);
}

int main(int argc, char** argv)
{
  //initialize ROS
  ros::init(argc, argv, "odom");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("omni_odom", 60);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::TwistStamped>("vel", 60);
  ros::Subscriber pose = n.subscribe("pose_stamped", 10, PoseCallBack);
  
  //initialize step displacement variables
  double x_last = 0.0;
  double y_last = 0.0;
  double th_last = 0.0;
  
  //initialize total displacement variables
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  
  ros::Time time_last = ros::Time::now();
  //ros::Time time_new = time_last;
  
  // Spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  //counter to skip the first run
  int count = 0;

  while(ros::ok()) 
  { 
    //calculates the time between messages being sent
    float dt = (time_new - time_last).toSec();
    
    //info
    ROS_WARN("x_new: %lf, y_new: %lf, time_new: %lf", x_new, y_new, time_new.toSec());
    ROS_WARN("x_last: %lf, y_last: %lf, time_last: %lf", x_last, y_last, time_last.toSec());
    ROS_WARN("dt: %lf", dt);

    //skips the rest of the loop if for some reason no time has passed between encoder counts
    if(dt == 0 || count == 0)
    {
    }
    else
    {
      //compute the change in displacement
      double delta_x = x_new - x_last;
      double delta_y = y_new - y_last;
      double delta_th = th_new - th_last;
    
      //compute the velocity
      double vx = delta_x / dt;
      double vy = delta_y / dt;
      double vth = delta_th / dt;

      //compute the overall displacement
      x += delta_x;
      y += delta_y;
      th += delta_th;

      //create quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

      //the transform is being published by laser_scan_matcher

      //publish the odometry
      nav_msgs::Odometry odom;
      odom.header.stamp = time_new;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the covariance
      odom.pose.covariance[0] = 0.2; //change to 1e3 if this is still bad
      odom.pose.covariance[7] = 0.2; //change to 1e3 if this is still bad 
      odom.pose.covariance[14] = 1e100;
      odom.pose.covariance[21] = 1e100;
      odom.pose.covariance[28] = 1e100;
      odom.pose.covariance[35] = 0.2; //change to 1e3 if this is still bad

      //set the velocity
      odom.child_frame_id = "base_footprint";
      odom.twist.twist.linear.x = vx; 
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = vth;

      //publish the message
      odom_pub.publish(odom);
      
      //publish the velocities as geometry_msgs::TwistStamped
      geometry_msgs::TwistStamped vel;
      vel.header.stamp = time_new;
      vel.header.frame_id = "odom";
      vel.twist.linear.x = vx;
      vel.twist.linear.y = vy;
      vel.twist.linear.z = 0;
      vel.twist.angular.x = 0;
      vel.twist.angular.y = 0;
      vel.twist.angular.z = vth;
      vel_pub.publish(vel);
    }
    
    //save previous values
    time_last = time_new;
    x_last = x_new;
    y_last = y_new;
    th_last = th_new;
    
    count = 1;
  }
  
  spinner.stop();
    
  return 0;
}
