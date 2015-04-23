#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

sensor_msgs::LaserScan newScan;

//function to save the new pose
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  newScan.header = msg->header;
  newScan.angle_min = msg->angle_min;
  newScan.angle_max = msg->angle_max;
  newScan.angle_increment = msg->angle_increment;
  newScan.time_increment = msg->time_increment;
  newScan.scan_time = msg->scan_time;
  newScan.range_min = msg->range_min;
  newScan.range_max = msg->range_max;
  newScan.ranges = msg->ranges;
  newScan.intensities = msg->intensities;
}

int main(int argc, char** argv)
{
  //initialize ROS
  ros::init(argc, argv, "republish_scans");
  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 60);
  ros::Subscriber front_scan = n.subscribe("/front_laser/scan", 30, laser_callback);
  ros::Subscriber rear_scan = n.subscribe("/rear_laser/scan", 30, laser_callback);

  // Spinner
  ros::AsyncSpinner spinner(2);
  spinner.start();

  while(ros::ok()) 
  { 
    scan_pub.publish(newScan);
  }
  
  spinner.stop();
    
  return 0;
}
