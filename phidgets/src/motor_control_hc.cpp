/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets motor control HC
 *  Copyright (c) 2010, Bob Mottram
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <phidget21.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "phidgets/motor_params.h"

// handle
CPhidgetMotorControlHandle phid;

// motor controller state publisher
ros::Publisher motors_pub;

float speed = 20;
float acceleration = 20;
bool x_forward = true;
bool invert_rotation = false;
bool invert_forward = false;
double rotation_offset = 0;

nav_msgs::Odometry odom;
double current_linear_velocity = 0;
double current_angular_velocity = 0;
double linear_velocity_proportional = 0.1;
double linear_velocity_integral = 0.0;
double linear_velocity_derivative = 0.0;
double angular_velocity_proportional = 0.1;
double angular_velocity_integral = 0.0;
double angular_velocity_derivative = 0.0;
double max_angular_velocity = 0.1;
double linear_deadband = 0.02;
double angular_deadband = 0.02;
double max_angular_error = 10*3.1415927/180.0;
double max_velocity_error = 0.05;
double max_angular_accel = 0.5;
double max_linear_accel = 0.3;
double ITerm[2];
double last_v=0,last_angv=0;

bool odometry_active = false;

ros::Time last_velocity_command;

bool motors_active = false;
bool initialised = false;

int AttachHandler(CPhidgetHandle phid, void *userptr)
{
  int serial_number;
  const char *name;

  CPhidget_getDeviceName (phid, &name);
  CPhidget_getSerialNumber(phid, &serial_number);
  ROS_INFO("%s Serial number %d attached!", name, serial_number);

  return 0;
}

int DetachHandler(CPhidgetHandle phid, void *userptr)
{
  int serial_number;
  const char *name;

  CPhidget_getDeviceName (phid, &name);
  CPhidget_getSerialNumber(phid, &serial_number);
  ROS_INFO("%s Serial number %d detached!", name, serial_number);

  return 0;
}

int ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *Description)
{
  ROS_INFO("Error handled. %d - %s", ErrorCode, Description);
  return 0;
}

int InputChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State)
{
  phidgets::motor_params m;
  m.index = Index;
  m.value_type = 1;
  m.value = (float)State;
  if (initialised) motors_pub.publish(m);
  //ROS_INFO("Motor input %d Inputs %d", Index, State);
  return 0;
}

int VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
{
  phidgets::motor_params m;
  m.index = Index;
  m.value_type = 2;
  m.value = (float)Value;
  if (initialised) motors_pub.publish(m);
  //ROS_INFO("Motor %d Velocity %.2f", Index, (float)Value);
  return 0;
}

int CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
{
  phidgets::motor_params m;
  m.index = Index;
  m.value_type = 3;
  m.value = (float)Value;
  if (initialised) motors_pub.publish(m);
  //ROS_INFO("Motor %d Current %.2f", Index, (float)Value);
  return 0;
}

int display_properties(CPhidgetMotorControlHandle phid)
{
  int serial_number, version, num_motors, num_inputs;
  const char* ptr;

  CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
  CPhidget_getSerialNumber((CPhidgetHandle)phid,
						 &serial_number);
  CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

  CPhidgetMotorControl_getInputCount(phid, &num_inputs);
  CPhidgetMotorControl_getMotorCount(phid, &num_motors);

  ROS_INFO("%s", ptr);
  ROS_INFO("Serial Number: %d", serial_number);
  ROS_INFO("Version: %d", version);
  ROS_INFO("Number of motors %d", num_motors);
  ROS_INFO("Number of inputs %d", num_inputs);

  return 0;
}

bool attach(CPhidgetMotorControlHandle &phid, int serial_number)
{
  // create the object
  CPhidgetMotorControl_create(&phid);

  // Set the handlers to be run when the device is
	// plugged in or opened from software, unplugged
	// or closed from software, or generates an error.
  CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid, AttachHandler, NULL);
  CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid, DetachHandler, NULL);
  CPhidget_set_OnError_Handler((CPhidgetHandle)phid, ErrorHandler, NULL);

  // Registers a callback that will run if an input changes.
  // Requires the handle for the Phidget, the function
	// that will be called, and a arbitrary pointer that
	// will be supplied to the callback function (may be NULL).
  CPhidgetMotorControl_set_OnInputChange_Handler (phid, InputChangeHandler, NULL);

  // Registers a callback that will run if a motor changes.
  // Requires the handle for the Phidget, the function
	// that will be called, and a arbitrary pointer that
	// will be supplied to the callback function (may be NULL).
  CPhidgetMotorControl_set_OnVelocityChange_Handler (phid, VelocityChangeHandler, NULL);

  // Registers a callback that will run if the current
	// draw changes.
  // Requires the handle for the Phidget, the function
	// that will be called, and a arbitrary pointer that
	// will be supplied to the callback function (may be NULL).
  CPhidgetMotorControl_set_OnCurrentChange_Handler (phid, CurrentChangeHandler, NULL);

  //open the device for connections
  CPhidget_open((CPhidgetHandle)phid, serial_number);

  // get the program to wait for an motor control
	// device to be attached
  if (serial_number == -1) 
  {
    ROS_INFO("Waiting for Motor Control HC Phidget to be attached....");
  }
  else 
  {
    ROS_INFO("Waiting for Motor Control HC Phidget %d to be attached....", serial_number);
  }
  int result;
  if((result = CPhidget_waitForAttachment((CPhidgetHandle)phid, 10000)))
	{
		const char *err;
		CPhidget_getErrorDescription(result, &err);
		ROS_ERROR("Problem waiting for motor attachment: %s", err);
			return false;
	}
  else return true;
}

/*!
 * \brief disconnect the motor controller
 */
void disconnect(CPhidgetMotorControlHandle &phid)
{
  ROS_INFO("Closing...");
  CPhidget_close((CPhidgetHandle)phid);
  CPhidget_delete((CPhidgetHandle)phid);
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& ptr)
{
  if (initialised) 
  {
    nav_msgs::Odometry o = *ptr;

    odom.header.stamp = o.header.stamp;
    odom.header.frame_id = o.header.frame_id;
    odom.pose.pose.position.x = o.pose.pose.position.x;
    odom.pose.pose.position.y = o.pose.pose.position.y;
    odom.pose.pose.position.z = o.pose.pose.position.z;
    odom.pose.pose.orientation = o.pose.pose.orientation;
    odom.child_frame_id = o.child_frame_id;
    odom.twist.twist.linear.x = o.twist.twist.linear.x;
    odom.twist.twist.linear.y = o.twist.twist.linear.y;
    odom.twist.twist.angular.z = o.twist.twist.angular.z;

    odometry_active = true;
  }
}

/*!
 * \brief callback when a velocity command is received
 * \param ptr encoder parameters
 */
void velocityCommandCallback(const geometry_msgs::Twist::ConstPtr& ptr)
{
  if (initialised) 
  {
    geometry_msgs::Twist m = *ptr;
    // convert Twist to motor velocities
    float rotate,forward;
    double incr,dInput;

    float x = m.linear.x;
    float y = m.angular.z;

    if ((m.linear.x!=0) || (m.angular.z!=0)) 
    {
      //ROS_DEBUG("cmd_vel %.3f angular %.3f", m.linear.x,m.angular.z);
      //ROS_DEBUG("actual  %.3f/%.3f angular %.3f", odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z);
    }
    else 
    {
      //ROS_DEBUG("Zero speed");
    }

    if (x_forward) 
    {
      rotate = y;
      forward = x;
    }
    else 
    {
      rotate = x;
      forward = y;
    }
    float forward_speed = speed * forward;

    ros::Time current_time = ros::Time::now();
    if (odometry_active) 
    {
      if (motors_active) 
      {
        double time_elapsed = (current_time - odom.header.stamp).toSec();
                
        if (time_elapsed > 0.001) 
        {
          double angular_velocity_error = rotate - odom.twist.twist.angular.z;

          // how fast are we actually moving?
          double vx = odom.twist.twist.linear.x;
          double vy = odom.twist.twist.linear.y;
          double v = sqrt(vx*vx + vy*vy);
          if (forward<0) v=-v;

          // forward velocity error
          double forward_velocity_error = forward - v;

          double angular_accel = angular_velocity_error / time_elapsed;
          
          if (fabs(angular_accel) > max_angular_accel) 
          {
            ROS_DEBUG("Angular acceleration limit reached");
            
            if (angular_accel > 0) 
            {
              angular_accel = max_angular_accel;
            }
            else 
            {
              angular_accel = -max_angular_accel;
            }
          }

          if (fabs(angular_velocity_error) > max_angular_error) 
          {
            ROS_WARN("Angular velocity error %f degrees/sec = %f - %f", angular_velocity_error*180/3.1415927, rotate, odom.twist.twist.angular.z);
          }
          
          if ((rotate>0.1) && (odom.twist.twist.angular.z<-0.1)) 
          {
            ROS_WARN("Rotation command positive and odometry direction negative");
          }
          
          if ((rotate<-0.1) && (odom.twist.twist.angular.z>0.1)) 
          {
            ROS_WARN("Rotation command negative and odometry direction positive");
          }
                    
          if (fabs(forward_velocity_error) > max_velocity_error) 
          {
          }
          double linear_accel = forward_velocity_error / time_elapsed;
                    
          if (fabs(linear_accel) > max_linear_accel) 
          {
            ROS_WARN("Linear acceleration limit %f/%f reached", linear_accel,max_linear_accel);
            
            if (linear_accel > 0) 
            {
              linear_accel = max_linear_accel;
            }
            else 
            {
              linear_accel = -max_linear_accel;
            }
          }

          // linear integral and derivative
          ITerm[0] += linear_velocity_integral * linear_accel;
                    
          if (ITerm[0] > max_linear_accel) 
          {
            ITerm[0] = max_linear_accel;
          }
          else 
          {
            if (ITerm[0] < -max_linear_accel) 
            {
							ITerm[0] = -max_linear_accel;
						}
          }

          if (fabs(forward)>linear_deadband) 
          {
            incr = (linear_velocity_proportional * linear_accel) + ITerm[0] - (linear_velocity_derivative * (v - last_v));
            
            if (invert_forward) 
            {
              incr = -incr;
            }
            current_linear_velocity += incr;
          }
          else 
          {
            current_linear_velocity *= 0.8;
          }
          
          if (current_linear_velocity > speed) 
          {
						current_linear_velocity = speed;
					}
                    
          if (current_linear_velocity <-speed) 
          {
						current_linear_velocity = -speed;
					}

          // angular integral and derivative
          ITerm[1] += angular_velocity_integral * angular_accel;
                    
          if (ITerm[1] > max_angular_accel) 
          {
            ITerm[1] = max_angular_accel;
          }
          else 
          {
            if (ITerm[1] < -max_angular_accel) 
            {
							ITerm[1] = -max_angular_accel;
						}
          }

          if (fabs(rotate)>angular_deadband) 
          {
            incr = (angular_velocity_proportional * angular_accel) + ITerm[1] - (angular_velocity_derivative * (odom.twist.twist.angular.z - last_angv));
                        
            if (invert_rotation) 
            {
              incr = -incr;
            }
            current_angular_velocity += incr;
          }
          else 
          {
            current_angular_velocity *= 0.8;
          }
          
          if (current_angular_velocity > max_angular_velocity) 
          {
						current_angular_velocity = max_angular_velocity;
					}
                    
          if (current_angular_velocity < -max_angular_velocity) 
          {
						current_angular_velocity = -max_angular_velocity;
					}

          CPhidgetMotorControl_setVelocity (phid, 1, current_linear_velocity + current_angular_velocity);
          CPhidgetMotorControl_setVelocity (phid, 0, -current_linear_velocity + current_angular_velocity);
          CPhidgetMotorControl_setAcceleration (phid, 0, acceleration);
          CPhidgetMotorControl_setAcceleration (phid, 1, acceleration);

          last_v = v;
          last_angv = odom.twist.twist.angular.z;
        }
      }
    }
    else 
    {
      CPhidgetMotorControl_setVelocity (phid, 0, -forward_speed - (rotate*speed));
      CPhidgetMotorControl_setVelocity (phid, 1, forward_speed - (rotate*speed));
      CPhidgetMotorControl_setAcceleration (phid, 0, acceleration);
      CPhidgetMotorControl_setAcceleration (phid, 1, acceleration);
    }
    last_velocity_command = current_time;
    motors_active = true;
  }
}

void stop_motors()
{
    CPhidgetMotorControl_setVelocity (phid, 0, 0);
    CPhidgetMotorControl_setVelocity (phid, 1, 0);
    motors_active = false;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_motor_control_hc");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    int serial_number = -1;
    nh.getParam("serial", serial_number);
    std::string name = "motorcontrol";
    nh.getParam("name", name);
    nh.getParam("x_forward", x_forward);
    nh.getParam("rotation", rotation_offset);
    std::string odometry_topic = "odom";
    nh.getParam("odometry", odometry_topic);
    double g = 0;
    const double min_g = 0.00001;
    nh.getParam("max_angular_error", g);
    if (g>min_g) max_angular_error = g;
    g=0;
    nh.getParam("max_velocity_error", g);
    if (g>min_g) max_velocity_error = g;
    g=0;
    nh.getParam("linear_deadband", g);
    if (g>min_g) linear_deadband = g;
    g=0;
    nh.getParam("angular_deadband", g);
    if (g>min_g) angular_deadband = g;
    g=0;
    nh.getParam("linear_proportional", g);
    if (g>min_g) linear_velocity_proportional = g;
    g=0;
    nh.getParam("linear_integral", g);
    if (g>min_g) linear_velocity_integral = g;
    g=0;
    nh.getParam("linear_derivative", g);
    if (g>min_g) linear_velocity_derivative = g;
    g=0;
    nh.getParam("angular_proportional", g);
    if (g>min_g) angular_velocity_proportional = g;
    g=0;
    nh.getParam("angular_integral", g);
    if (g>min_g) angular_velocity_integral = g;
    g=0;
    nh.getParam("angular_derivative", g);
    if (g>min_g) angular_velocity_derivative = g;
    g=0;
    nh.getParam("max_angular_velocity", g);
    if (g>min_g) max_angular_velocity = g;
    nh.getParam("invert_rotation", invert_rotation);
    nh.getParam("invert_forward", invert_forward);
    if (serial_number==-1) {
        nh.getParam("serial_number", serial_number);
    }

    g=0;
    nh.getParam("max_angular_accel", g);
    if (g>0) max_angular_accel = g;
    g=0;
    nh.getParam("max_linear_accel", g);
    if (g>0) max_linear_accel = g;

    std::string topic_path = "phidgets/";
    nh.getParam("topic_path", topic_path);
    int timeout_sec = 2;
    nh.getParam("timeout", timeout_sec);
    int v=0;
    nh.getParam("speed", v);
    if (v>0) speed = v;
    int frequency = 30;
    nh.getParam("frequency", frequency);

    ITerm[0]=0;
    ITerm[1]=0;

    if (attach(phid, serial_number)) {
		display_properties(phid);

        const int buffer_length = 100;        
        std::string topic_name = topic_path + name;
        std::string service_name = name;
        if (serial_number > -1) {
            char ser[10];
            sprintf(ser,"%d", serial_number);
            topic_name += "/";
            topic_name += ser;
            service_name += "/";
            service_name += ser;
        }
        motors_pub =
			n.advertise<phidgets::motor_params>(topic_name,
												buffer_length);

        // receive velocity commands
        ros::Subscriber command_velocity_sub =
			n.subscribe("cmd_vel", 1, velocityCommandCallback);

        // subscribe to odometry
        ros::Subscriber odometry_sub =
			n.subscribe(odometry_topic, 1, odometryCallback);

        initialised = true;
        ros::Rate loop_rate(frequency);

        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();

            // SAFETY FEATURE
            // if a velocity command has not been received
			// for a period of time then stop the motors
            double time_since_last_command_sec =
				(ros::Time::now() -
				 last_velocity_command).toSec();
            if ((motors_active) &&
				(time_since_last_command_sec > timeout_sec)) {
                stop_motors();        
                ROS_WARN("No velocity command received - " \
						 "motors stopped");        
            }
        }

        disconnect(phid);
    }
    return 0;
}

