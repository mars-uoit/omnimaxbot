#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define DEG2RAD M_PI/180.0

ros::Publisher scan_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  sensor_msgs::LaserScan newScan;

  newScan.header.seq = scan->header.seq;
  newScan.header.stamp = scan->header.stamp;
  newScan.header.frame_id = scan->header.frame_id;

  double angle_min = -90 * DEG2RAD;
  newScan.angle_min = angle_min;
  double angle_max = scan->angle_max;
  newScan.angle_max = angle_max;
  double angle_increment = scan->angle_increment;
  newScan.angle_increment = angle_increment;

  newScan.time_increment = scan->time_increment;

  newScan.scan_time = scan->scan_time;

  newScan.range_min = scan->range_min;
  newScan.range_max = scan->range_max;

  int orig_no_inc = (scan->angle_max - scan->angle_min) / scan->angle_increment;
  int new_no_inc = (angle_max - angle_min) / angle_increment;
  int drop = orig_no_inc - new_no_inc;

  //indexes 0-539, want to drop the first 90 (for now, may fine tune later), so index 90-539
  for (int i = drop; i < orig_no_inc; i++)
  {
    newScan.ranges[i-drop] = scan->ranges[i];
    newScan.intensities[i-drop] = scan->intensities[i];
  }

  scan_pub.publish(scan);
}

int main(int argc, char** argv)
{
  // node setup
  ros::init(argc, argv, "rear_scan_limiter");
  ros::NodeHandle n;
  
  // publisher setup
  scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);
  
  // subscriber setup
  ros::Subscriber scans = n.subscribe<sensor_msgs::LaserScan>("/rear_laser/scan", 10, scanCallback);
  
  // Spinner
  ros::AsyncSpinner spinner(2);
  spinner.start();

  while(ros::ok()) 
  {
    ros::spinOnce();
  }

  spinner.stop();
  
  return 0;
}
