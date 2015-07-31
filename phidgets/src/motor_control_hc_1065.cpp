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
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include "phidgets/motor_params.h"
#include "phidgets/encoder_params.h"

// handle
CPhidgetMotorControlHandle phid;

// motor controller state publisher
ros::Publisher motors_pub;

// encoder count publisher
ros::Publisher encoder_pub;

// goal flag publisher
ros::Publisher goal_pub;

ros::Time last_command;

float acceleration = 20;
float speed = 62.5;
bool x_forward = true;
bool invert_rotation = false;
bool invert_forward = false;
double rotation_offset = 0;
bool timedOut = false;

double errorLast = 0;
double integral = 0;
double derivative = 0; 
double Kp = 0.8;
double Ki = 0.5;
double Kd = 0.1;
int deadband = 3071;

int position = 0;
int targetPosition = 0;

bool odometry_active = false;

bool motors_active = false;
bool initialised = false;

void stop_motors()
{
  CPhidgetMotorControl_setVelocity (phid, 0, 0);
  CPhidgetMotorControl_setBraking (phid, 0, 100);
  motors_active = false;
}

void start_motors(double duty_cycle)
{
  CPhidgetMotorControl_setBraking (phid, 0, 0);
  CPhidgetMotorControl_setVelocity (phid, 0, duty_cycle);
  CPhidgetMotorControl_setAcceleration (phid, 0, acceleration);
  motors_active = true;
}

void PID(int actualPosition)
{
  std_msgs::Bool goalReached;
  goalReached.data = false;
  double dt = 0.008;

  if(timedOut == false)
  {
    double error = targetPosition - actualPosition;
    double duty_cycle = (Kp * error) + (Ki * integral) + (Kd * derivative);
 
    if (duty_cycle > 100)
    {
      duty_cycle = 100;
    }
    else if (duty_cycle < -100)
    {
      duty_cycle = -100;
    }
    else
    {
      integral += (error * dt);
    }
 
    derivative = (error - errorLast)/dt;
 
    //ROS_INFO_STREAM("error: " << error);
    //ROS_INFO_STREAM("integral: " << integral);
    //ROS_INFO_STREAM("derivative: " << derivative);
    //ROS_INFO_STREAM("errorLast: " << errorLast); 

    //derivative = (error - errorLast)/dt;
    errorLast = error;

    if (duty_cycle == 0)
    {
      stop_motors();
    }
    else if ((error > 0 && error <= deadband) || (error < 0 && error > -deadband))
    {
      stop_motors();
      integral = 0;
      derivative = 0;
      goalReached.data = true;
    }
    else
    {
      start_motors(duty_cycle);
    }
    ROS_INFO_STREAM("duty_cycle: " << duty_cycle);
    goal_pub.publish(goalReached);
  }
}

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

int EncoderUpdateHandler(CPhidgetMotorControlHandle MC, void *userPtr, int Index, int positionChange)
{
  position -= positionChange; 

  ROS_INFO_STREAM("position: " << position);

  phidgets::encoder_params m;
  m.index = Index;
  m.count = position;
  m.count_change = positionChange;
  m.time = 0.008;

  encoder_pub.publish(m);

  PID(position);

  return 0;
}

int display_properties(CPhidgetMotorControlHandle phid)
{
  int serial_number, version, num_motors, num_inputs;
  const char* ptr;

  CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
  CPhidget_getSerialNumber((CPhidgetHandle)phid, &serial_number);
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

  // Registers a callback that will run when the encoder position in updated
  // Requires a handle for the Phidget, the function
  // that will be called, and an arbitrary pointer that
  // will be supplied to the callback function (may be NULL).
  CPhidgetMotorControl_set_OnEncoderPositionUpdate_Handler (phid, EncoderUpdateHandler, NULL);

  //open the device for connections
  CPhidget_open((CPhidgetHandle)phid, serial_number);

  // get the program to wait for an motor control
	// device to be attached
  if (serial_number == -1) 
  {
    ROS_INFO("Waiting for Motor Control 1065 Phidget to be attached....");
  }
  else 
  {
    ROS_INFO("Waiting for Motor Control 1065 Phidget %d to be attached....", serial_number);
  }
  int result;
  if((result = CPhidget_waitForAttachment((CPhidgetHandle)phid, 10000)))
  {
    const char *err;
    CPhidget_getErrorDescription(result, &err);
    ROS_ERROR("Problem waiting for motor attachment: %s", err);
    return false;
  }
  else
  {
    ROS_INFO("Motor attached");
    return true;
  }
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
/*
void positionCommandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  target_position = msg->linear.x;
}
*/

void positionCommandCallback(const std_msgs::Float32::ConstPtr& msg)
{
  last_command = ros::Time::now();
  double rawPosition = msg->data; //take in the change in height required in m
  double conv = 39.3700787; //inchs per metre
  double gear_ratio = 13.0;
  double TPI = 10.0; //threads per inch
  double cpr = 300.0; //from encoder

  double unroundedGoal = rawPosition * conv * gear_ratio * TPI * cpr;
  int hold = (int)unroundedGoal;

  ROS_INFO_STREAM("hold = " << hold);

  if(unroundedGoal > 0 && unroundedGoal - hold >= 0.5)
    hold += 1;
  else if (unroundedGoal < 0 && unroundedGoal - hold <= -0.5)
    hold -= 1;

  targetPosition = hold;
  timedOut = false;
  ROS_INFO_STREAM("targetPosition = " << targetPosition);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "phidgets_motor_control_hc_1065");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  int serial_number = -1;
  nh.getParam("serial", serial_number);
  std::string name = "motorcontrol";
  nh.getParam("name", name);
  nh.getParam("x_forward", x_forward);
  nh.getParam("rotation", rotation_offset);

  if (serial_number==-1) 
  {
    nh.getParam("serial_number", serial_number);
  }

  std::string topic_path = "phidgets/";
  nh.getParam("topic_path", topic_path);
  int timeout_sec = 2;
  nh.getParam("timeout", timeout_sec);
  int v=0;
  nh.getParam("speed", v);
  if (v>0) speed = v;
  int frequency = 30;
  nh.getParam("frequency", frequency);

  if (attach(phid, serial_number)) 
  {
    display_properties(phid);

    const int buffer_length = 100;        
    std::string topic_name = topic_path + name;
    std::string service_name = name;
    if (serial_number > -1) 
    {
      char ser[10];
      sprintf(ser,"%d", serial_number);
      topic_name += "/";
      topic_name += ser;
      service_name += "/";
      service_name += ser;
    }
    motors_pub = n.advertise<phidgets::motor_params>(topic_name, buffer_length);

    // receive position commands
    ros::Subscriber position_sub = n.subscribe("fork_position", 1, positionCommandCallback);
    
    // publish encoder counts
    encoder_pub = n.advertise<phidgets::encoder_params>("fork_encoder", 5);

    // publish encoder counts
    goal_pub = n.advertise<std_msgs::Bool>("fork_goal_reached", 1);

    last_command = ros::Time::now();

    initialised = true;
    ros::Rate loop_rate(frequency);

    while (ros::ok()) 
    {
      loop_rate.sleep();
      ros::spinOnce();
      
      // SAFETY FEATURE
      // if a velocity command has not been received
      // for a period of time then stop the motors
      double time_since_last_command_sec = (ros::Time::now() - last_command).toSec();
      if ((motors_active) && (time_since_last_command_sec > timeout_sec)) 
      {
        stop_motors();
        ROS_WARN("No velocity command received - motors stopped");
        timedOut = true;
      }
    }

    //disconnect(phid);
  }
  disconnect(phid);
  return 0;
}

