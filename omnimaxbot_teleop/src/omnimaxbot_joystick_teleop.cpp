#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

ros::Publisher vel_pub;
double x_drive_scale = 2.0;
double y_drive_scale = 2.0;
double turn_scale = 1.0;

double plan_x = 0;
double plan_y = 0;
double plan_th = 0;

void planCallback(const geometry_msgs::Twist::ConstPtr& plan)
{
  plan_x = plan->linear.x;
  plan_y = plan->linear.y;
  plan_th = plan->angular.z;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  int linear_y = 0;
  int linear_x = 1;
  int angular_th = 2;
  int deadman = 0;
  int deadman_button = 0;
  int plan_button = 0;
  int plan = 1;
  
  geometry_msgs::Twist vel;
  
  deadman_button = joy->buttons[deadman];
  plan_button = joy->buttons[plan];
  
  if(deadman_button == 1)
  {
    vel.linear.x = joy->axes[linear_x] * x_drive_scale;
    vel.linear.y = joy->axes[linear_y] * y_drive_scale;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = joy->axes[angular_th] * turn_scale;
  }
  else if(plan_button == 1)
  {
    vel.linear.x = plan_x;
    vel.linear.y = plan_y;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = plan_th;
  }
  else
  {
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;
  }
  
  vel_pub.publish(vel); 
}

int main(int argc, char** argv)
{
  // node setup
  ros::init(argc, argv, "omnibot_joystick_teleop");
  ros::NodeHandle n;
  
  // get parameters
  n.param("turn_scale", turn_scale, 1.0);
  n.param("x_drive_scale", x_drive_scale, 2.0);
  n.param("y_drive_scale", y_drive_scale, 2.0);
  
  // publisher setup
  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  // subscriber setup
  ros::Subscriber planned_cmd_vel = n.subscribe<geometry_msgs::Twist>("omni_cmd_vel", 10, planCallback);
  ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);
  
  ros::spin();
  
  return 0;
}
