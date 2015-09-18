#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace std;

//initialize publishers
ros::Publisher front_cmd_vel_pub;
ros::Publisher rear_cmd_vel_pub;

double wheel_radius = 0.125;

// Max speed = 1.77 m/s = 6.37 km/hr
// Limiting speed to 1.5 m/s
// That is 114.59 rpm, roughly 107 input

static double REL_MAX = 35.0;
static int GEAR_RATIO = 20;

//function to calculate the wheel velocities
void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
  //the sum of the distance between the wheel's x-coordinate and the x-axis, and the y-coordinate and the y-axis
  const double k = 0.47 + 0.55;
	
  // Copy the velocities (m/s)
  double Vx = msg->linear.x;
  double Vy = msg->linear.y;
  double Wv = msg->angular.z;
	
  // Calculate individual wheel linear velocities (m/s)
  double W1 = Vx - Vy + Wv * -k; 
  double W2 = Vx + Vy + Wv * k;
  double W3 = Vx + Vy + Wv * -k; 
  double W4 = Vx - Vy + Wv * k; 
	
  // Convert linear velocities (m/s) to rotational velocities (rpm)
  double front_B_rpm = W1 * (60.0 / (2 * M_PI * wheel_radius)) * GEAR_RATIO;
  double front_A_rpm = W2 * (60.0 / (2 * M_PI * wheel_radius)) * GEAR_RATIO;
  double rear_B_rpm = W3 * (60.0 / (2 * M_PI * wheel_radius)) * GEAR_RATIO;
  double rear_A_rpm = W4 * (60.0 / (2 * M_PI * wheel_radius)) * GEAR_RATIO;
    
  // Convert from rpm to relative 
  // rel = (rpm * PPR * Time Base+1)/58593.75(from pg 81 of the Operating Manual)
  double front_A_rel = front_A_rpm * 250 * 11 / 58593.75;
  double front_B_rel = front_B_rpm * 250 * 11 / 58593.75;
  double rear_A_rel = rear_A_rpm * 250 * 11 / 58593.75;
  double rear_B_rel = rear_B_rpm * 250 * 11 / 58593.75;

  // Bounds check
  if(front_A_rel > REL_MAX || front_A_rel < -REL_MAX)
  {
    double rel = abs(REL_MAX / front_A_rel);
    front_A_rel = front_A_rel * rel;
    front_B_rel = front_B_rel * rel;
    rear_A_rel = rear_A_rel * rel;
    rear_B_rel = rear_B_rel * rel;
  }
	
  if(front_B_rel > REL_MAX || front_B_rel < -REL_MAX)
  {
    double rel = abs(REL_MAX / front_B_rel);
    front_A_rel = front_A_rel * rel;
    front_B_rel = front_B_rel * rel;
    rear_A_rel = rear_A_rel * rel;
    rear_B_rel = rear_B_rel * rel;
  }
	
  if(rear_A_rel > REL_MAX || rear_A_rel < -REL_MAX)
  {
    double rel = abs(REL_MAX / rear_A_rel);
    front_A_rel = front_A_rel * rel;
    front_B_rel = front_B_rel * rel;
    rear_A_rel = rear_A_rel * rel;
    rear_B_rel = rear_B_rel * rel;
  }
	
  if(rear_B_rel > REL_MAX || rear_B_rel < -REL_MAX)
  {
    double rel = abs(REL_MAX / rear_B_rel);
    front_A_rel = front_A_rel * rel;
    front_B_rel = front_B_rel * rel;
    rear_A_rel = rear_A_rel * rel;
    rear_B_rel = rear_B_rel * rel;
  }

  // publish motor speeds
  geometry_msgs::Twist front_cmd_vel;
  front_cmd_vel.linear.x = front_A_rel * -1;
  front_cmd_vel.linear.y = front_B_rel;
  front_cmd_vel.linear.z = 0.0;
  front_cmd_vel.angular.x = 0.0;
  front_cmd_vel.angular.y = 0.0;
  front_cmd_vel.angular.z = 0.0;

  geometry_msgs::Twist rear_cmd_vel;
  rear_cmd_vel.linear.x = rear_A_rel * -1;
  rear_cmd_vel.linear.y = rear_B_rel;
  rear_cmd_vel.linear.z = 0.0;
  rear_cmd_vel.angular.x = 0.0;
  rear_cmd_vel.angular.y = 0.0;
  rear_cmd_vel.angular.z = 0.0;
  
  front_cmd_vel_pub.publish(front_cmd_vel);
  rear_cmd_vel_pub.publish(rear_cmd_vel);
}

int main(int argc, char **argv) 
{ 
  // Node setup
  ros::init(argc, argv, "omni_cmd_vel");
  ros::NodeHandle n;

  // cmd_vel publishers
  front_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/omnimaxbot/front/cmd_vel", 5);
  rear_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/omnimaxbot/rear/cmd_vel", 5);

  // cmd_vel Subscriber
  ros::Subscriber sub = n.subscribe("/cmd_vel", 5, cmd_velCallback);

  ros::spin();
  
  return 0;
}
