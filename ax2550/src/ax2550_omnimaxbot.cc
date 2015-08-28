#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ax2550/StampedEncoders.h"
#include <string>
#include <cmath>
#include "ax2550/ax2550.h"

using namespace ax2550;
using std::string;

AX2550 *mc;
ros::Publisher encoder_pub;

double encoder_poll_rate;
std::string odom_frame_id;
size_t error_count;
double target_speed_right = 0.0;
double target_speed_left = 0.0;

double rot_cov = 0.0;
double pos_cov = 0.0;

const int timeout_sec = 1.0;
ros::Time time_last;

// Persistent variables
double prev_x = 0, prev_y = 0, prev_w = 0;
ros::Time prev_time;

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
  time_last = ros::Time::now();
  if(mc == NULL || !mc->isConnected())
    return;
  
  //set the targets
  target_speed_right = msg->linear.x;
  target_speed_left = msg->linear.y;
}

void controlLoop() 
{
  // ROS_INFO("Relative move commands: %f %f", target_speed, target_direction);
  try 
  {
    //SAFETY TIME OUT
    //if a command is not received for 1 seconds the motors time out
    double time_past = (ros::Time::now() - time_last).toSec();
    if(time_past > timeout_sec)
    {
      mc->move(0,0);
      ROS_WARN("No velocity commands received - motors timed out");
    }
    else
    {
      mc->move(target_speed_right, target_speed_left);
    }
  } 
  catch(const std::exception &e) 
  {
    if (string(e.what()).find("did not receive") != string::npos || string(e.what()).find("failed to receive an echo") != string::npos) 
    {
      ROS_WARN("Error commanding the motors: %s", e.what());
    } 
    else 
    {
      ROS_ERROR("Error commanding the motors: %s", e.what());
      mc->disconnect();
    }
  }
}

void errorMsgCallback(const std::string &msg) 
{
  ROS_ERROR("%s", msg.c_str());
}

void warnMsgCallback(const std::string &msg) 
{
  ROS_WARN("%s", msg.c_str());
}

void infoMsgCallback(const std::string &msg) 
{
  ROS_INFO("%s", msg.c_str());
}

void debugMsgCallback(const std::string &msg) 
{
  ROS_DEBUG("%s", msg.c_str());
}

void queryEncoders() 
{
  // Make sure we are connected
  if(!ros::ok() || mc == NULL || !mc->isConnected())
    return;
    
  long encoder1, encoder2;
  ros::Time now = ros::Time::now();
  // Retreive the data
  try 
  {
    mc->queryEncoders(encoder1, encoder2, true);
    if (error_count > 0) 
    {
      error_count -= 1;
    }
  } 
  catch(std::exception &e) 
  {
    if (string(e.what()).find("failed to receive ") != string::npos && error_count != 10) 
    {
      error_count += 1;
      ROS_WARN("Error reading the Encoders: %s", e.what());    
    } 
    else 
    {
      ROS_ERROR("Error reading the Encoders: %s", e.what());
      mc->disconnect();
    }
    return;
  }
    
  double delta_time = (now - prev_time).toSec();
  prev_time = now;
  
  ax2550::StampedEncoders encoder_msg;
    
  encoder_msg.header.stamp = now;
  encoder_msg.header.frame_id = odom_frame_id;
  encoder_msg.encoders.time_delta = delta_time;
  encoder_msg.encoders.left_wheel = encoder2;
  encoder_msg.encoders.right_wheel = -encoder1;
    
  encoder_pub.publish(encoder_msg);
}

int main(int argc, char **argv) 
{
  // Node setup
  ros::init(argc, argv, "ax2550_omnimaxbot");
  ros::NodeHandle n;
  prev_time = ros::Time::now();
  
  // Serial port parameter
  std::string port;
  n.param("serial_port", port, std::string("/dev/ttyS0"));
  
  // Odom Frame id parameter
  n.param("odom_frame_id", odom_frame_id, std::string("odom"));

  // Load up some covariances from parameters
  n.param("rotation_covariance",rot_cov, 1.0);
  n.param("position_covariance",pos_cov, 1.0);
  
  // Setup Encoder polling
  n.param("encoder_poll_rate", encoder_poll_rate, 25.0); //default 25.0
  ros::Rate encoder_rate(encoder_poll_rate);
  
  // Encoder Publisher
  encoder_pub = n.advertise<ax2550::StampedEncoders>("/ax2550/encoders", 5);
  
  // cmd_vel Subscriber
  ros::Subscriber sub = n.subscribe("/cmd_vel", 1, cmd_velCallback);
  
  // Spinner
  ros::AsyncSpinner spinner(4);
  spinner.start();

  while(ros::ok()) 
  {
    ROS_INFO("AX2550 connecting to port %s", port.c_str());
    try 
    {
      mc = new AX2550();
      mc->warn = warnMsgCallback;
      mc->info = infoMsgCallback;
      mc->debug = debugMsgCallback;
      mc->connect(port);
    } 
    catch(std::exception &e) 
    {
      ROS_ERROR("Failed to connect to the AX2550: %s", e.what());
      if (mc != NULL) 
      {
      	mc->disconnect();
      }
    }
    int count = 0;
    while(mc != NULL && mc->isConnected() && ros::ok()) 
    {
      queryEncoders();
      if (count == 1) 
      {
        controlLoop();
        count = 0;
      } 
      else 
      {
        count += 1;
      }
      encoder_rate.sleep();
    }
    if (mc != NULL) 
    {
      delete mc;
    }
    mc = NULL;
    if(!ros::ok())
      break;
    ROS_INFO("Will try to reconnect to the AX2550 in 5 seconds.");
    for (int i = 0; i < 100; ++i) 
    {
      ros::Duration(5.0/100.0).sleep();
      if (!ros::ok())
        break;
    }
    target_speed_right = 0.0;
    target_speed_left = 0.0;
  }

  spinner.stop();
    
  return 0;
}
