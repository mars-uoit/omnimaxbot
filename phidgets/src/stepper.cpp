/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets stepper motor controller
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
#include <std_msgs/String.h>
#include "phidgets/stepper_params.h"

// handle
CPhidgetStepperHandle phid;

// stepper controller state publisher
ros::Publisher stepper_pub;

bool initialised = false;

int AttachHandler(CPhidgetHandle phid, void *userptr)
{
    int serial_number;
    const char *name;

    CPhidget_getDeviceName (phid, &name);
    CPhidget_getSerialNumber(phid, &serial_number);
    ROS_INFO("%s Serial number %d attached!",
			 name, serial_number);

    return 0;
}

int DetachHandler(CPhidgetHandle phid, void *userptr)
{
    int serial_number;
    const char *name;

    CPhidget_getDeviceName (phid, &name);
    CPhidget_getSerialNumber(phid, &serial_number);
    ROS_INFO("%s Serial number %d detached!",
			 name, serial_number);

    return 0;
}

int ErrorHandler(CPhidgetHandle phid, void *userptr,
				 int ErrorCode, const char *Description)
{
    ROS_INFO("Error handled. %d - %s", ErrorCode, Description);
    return 0;
}

int PositionChangeHandler(CPhidgetStepperHandle stepper,
						  void *usrptr, int Index,
						  long long Value)
{
    if (initialised) {
        phidgets::stepper_params m;
        m.index = Index;
        m.position = Value;
        stepper_pub.publish(m);
        ROS_INFO("Motor %d Current position %lld",
				 Index, Value);
    }
    return 0;
}

int display_properties(CPhidgetStepperHandle phid)
{
    int serial_number, version, num_motors;
    const char* ptr;

    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid,
							 &serial_number);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

    CPhidgetStepper_getMotorCount (phid, &num_motors);

    ROS_INFO("%s", ptr);
    ROS_INFO("Serial Number: %d", serial_number);
    ROS_INFO("Version: %d", version);
    ROS_INFO("Number of motors %d", num_motors);

    return 0;
}

bool attach(CPhidgetStepperHandle &phid,
			int serial_number)
{
    //create the object
    CPhidgetStepper_create(&phid);

    // Set the handlers to be run when the device is plugged
	// in or opened from software, unplugged or closed from
	// software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid,
								  AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid,
								  DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)phid,
								 ErrorHandler, NULL);

    // Registers a callback that will run when the motor
	// position is changed.
    // Requires the handle for the Phidget, the function
	// that will be called, and an arbitrary pointer that
	// will be supplied to the callback function (may be NULL).
    CPhidgetStepper_set_OnPositionChange_Handler(phid,
												 PositionChangeHandler,
												 NULL);

    //open the device for connections
    CPhidget_open((CPhidgetHandle)phid, serial_number);

    // get the program to wait for an stepper control
	//  device to be attached
    if (serial_number == -1) {
        ROS_INFO("Waiting for Stepper Motor Control " \
				 "Phidget to be attached....");
    }
    else {
        ROS_INFO("Waiting for Stepper Motor Control " \
				 "Phidget %d to be attached....",
				 serial_number);
    }
    int result;
    if ((result =
		 CPhidget_waitForAttachment((CPhidgetHandle)phid,
									10000))) {
        const char *err;
        CPhidget_getErrorDescription(result, &err);
		ROS_ERROR("Problem waiting for attachment: %s", err);
		return false;
    }
    else return true;
}

/*!
 * \brief disconnect the stepper motor controller
 */
void disconnect(
				CPhidgetStepperHandle &phid)
{
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

// Request to change stepper position
void stepperCallback(const phidgets::stepper_params::ConstPtr& ptr)
{
    if (initialised) {
        phidgets::stepper_params s = *ptr;

        ROS_INFO("Motor %d Target position %lld Reference " \
				 "Velocity %.2f Reference Acceleration %.2f", 
				 s.index, s.position, s.velocity,
				 s.acceleration);

        int motor_index = (int)s.index;

        if (!s.engage) {
            CPhidgetStepper_setEngaged(phid, motor_index, 0);
        }
        if (s.reset_position) {
            CPhidgetStepper_setCurrentPosition(phid, 0, 0);
        }
        CPhidgetStepper_setAcceleration (phid, motor_index,
										 (double)s.acceleration);
        CPhidgetStepper_setVelocityLimit (phid, motor_index,
										  (double)s.velocity);
        CPhidgetStepper_setTargetPosition (phid, motor_index,
										   s.position);
        if (s.engage) {
            CPhidgetStepper_setEngaged(phid, motor_index, 1);
        }
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_stepper");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    int serial_number = -1;
    nh.getParam("serial", serial_number);
    std::string name = "stepper";
    nh.getParam("name", name);
    if (serial_number==-1) {
        nh.getParam("serial_number", serial_number);
    }
    std::string topic_path = "phidgets/";
    nh.getParam("topic_path", topic_path);

    int frequency = 30;
    nh.getParam("frequency", frequency);

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
        stepper_pub =
			n.advertise<phidgets::stepper_params>(topic_name,
												  buffer_length);

        // start service which can be used to set motor position
        ros::Subscriber sub;
        sub = n.subscribe(service_name, 1, stepperCallback);

        initialised = true;
        ros::Rate loop_rate(frequency);

        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        disconnect(phid);
    }
    return 0;
}

