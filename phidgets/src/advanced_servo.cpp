/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets advanced servo controller
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
#include "phidgets/servo_reference.h"
#include "phidgets/servo_params.h"

// Advanced servo handle
CPhidgetAdvancedServoHandle servo;

// servo state publisher
ros::Publisher servo_state_pub;

bool initialised = false;
int serial_number, version, numMotors=0;
bool *servo_engaged;


int AttachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
    int serialNo;
    const char *name;

    CPhidget_getDeviceName (ADVSERVO, &name);
    CPhidget_getSerialNumber(ADVSERVO, &serialNo);
    ROS_INFO("%s Serial number %d attached!", name, serialNo);

    return 0;
}

int DetachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
    int serialNo;
    const char *name;

    CPhidget_getDeviceName (ADVSERVO, &name);
    CPhidget_getSerialNumber(ADVSERVO, &serialNo);
    ROS_INFO("%s Serial number %d detached!", name, serialNo);

    return 0;
}

int ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr,
				 int ErrorCode, const char *Description)
{
    ROS_WARN("Error handled. %d - %s", ErrorCode, Description);
    return 0;
}

void position_changed(int Index, double Value)
{
    phidgets::servo_params s;
    s.index = (int)Index;
    s.position = (float)Value;
    //ROS_INFO("Servo position %f", s.position);
    if (initialised) servo_state_pub.publish(s);
}

int PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO,
						  void *usrptr, int Index, double Value)
{
    position_changed(Index,Value);
    return 0;
}

// Display the properties of the attached phidget to
// the screen.  We will be displaying the name,
// serial number and version of the attached device.
int display_properties(CPhidgetAdvancedServoHandle phid)
{
    const char* ptr;

    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid,
							 &serial_number);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

    CPhidgetAdvancedServo_getMotorCount (phid, &numMotors);

    ROS_INFO("%s", ptr);
    ROS_INFO("Serial Number: %d", serial_number);
    ROS_INFO("Version: %d", version);
    ROS_INFO("Motors: %d", numMotors);

    return 0;
}

bool attach(CPhidgetAdvancedServoHandle &servo,
			int serial_number)
{
    // create the advanced servo object
    CPhidgetAdvancedServo_create(&servo);

    // Set the handlers to be run when the device is
	// plugged in or opened from software, unplugged
	// or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo,
								  AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo,
								  DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)servo,
								 ErrorHandler, NULL);

    // Registers a callback that will run when the
	// motor position is changed.
    // Requires the handle for the Phidget, the
	// function that will be called, and an arbitrary
	// pointer that will be supplied to the
	// callback function (may be NULL).
    CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo,
													   PositionChangeHandler,
													   NULL);

    //open the device for connections
    CPhidget_open((CPhidgetHandle)servo, serial_number);

    // get the program to wait for an advanced
	// servo device to be attached
    if (serial_number == -1) {
        ROS_INFO("Waiting for Advanced Servo " \
				 "Controller Phidget to be attached....");
    }
    else {
        ROS_INFO("Waiting for Advanced Servo " \
				 "Controller Phidget %d to be attached....",
				 serial_number);
    }
    int result;
    if ((result =
		 CPhidget_waitForAttachment((CPhidgetHandle)servo,
									10000))) {
        const char *err;
        CPhidget_getErrorDescription(result, &err);
		ROS_ERROR("Problem waiting for attachment: %s", err);
		return false;
    }
    else return true;
}

/*!
 * \brief disconnect the servo controller
 */
void disconnect(CPhidgetAdvancedServoHandle &servo)
{
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)servo);
    CPhidget_delete((CPhidgetHandle)servo);
}

/*!
 * \brief set speed, acceleration and position of a servo
 * \param req requested parameters
 * \param res returned parameters
 */
bool servo_reference( phidgets::servo_reference::Request &req,
					 phidgets::servo_reference::Response &res)
{
    ROS_INFO("Servo %d Reference %.2f speed %.2f accel %.2f",
			 req.index, req.position,
			 req.speed, req.acceleration);

    int servo_index = (int)req.index;

    if (!req.engage) {
        if (servo_engaged[servo_index]) {
            CPhidgetAdvancedServo_setEngaged(servo,
											 servo_index, 0);
            servo_engaged[servo_index] = false;
        }
    }
    else {
        CPhidgetAdvancedServo_setAcceleration(servo,
											  servo_index,
											  (double)req.acceleration);
        CPhidgetAdvancedServo_setVelocityLimit(servo,
											   servo_index,
											   (double)req.speed);
        CPhidgetAdvancedServo_setPosition (servo,
										   servo_index,
										   (double)req.position);
        if ((req.engage) && (!servo_engaged[servo_index])) {
            CPhidgetAdvancedServo_setEngaged(servo,
											 servo_index, 1);
            servo_engaged[servo_index] = true;
        }
    }
    res.ack = 1;

    return(true);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_advanced_servo");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    int serial_number = -1;
    nh.getParam("serial", serial_number);
    std::string name = "servos";
    nh.getParam("name", name);
    if (serial_number==-1) {
        nh.getParam("serial_number", serial_number);
    }
    std::string topic_path = "phidgets/";
    nh.getParam("topic_path", topic_path);

    int frequency = 16;
    nh.getParam("frequency", frequency);

    if (attach(servo, serial_number)) {
		display_properties(servo);

        servo_engaged = new bool[numMotors];
        for (int i = 0; i < numMotors;i++) {
            servo_engaged[i]=false;
        }

        const int buffer_length = 10;        
        std::string topic_name = topic_path + name;
        std::string service_name = "servo_reference";
        if (serial_number > -1) {
            char ser[10];
            sprintf(ser,"%d", serial_number);
            topic_name += "/";
            topic_name += ser;
            service_name += "/";
            service_name += ser;
        }
        servo_state_pub =
			n.advertise<phidgets::servo_params>(topic_name,
												buffer_length);      
        ros::Rate loop_rate(frequency);

        // start service which can be used to
		// set reference values
        ROS_INFO("service_name %s", service_name.c_str());
        ros::ServiceServer service =
			n.advertiseService(service_name, servo_reference);

        initialised = true;

        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        disconnect(servo);
        delete [] servo_engaged;
    }
    return 0;
}

