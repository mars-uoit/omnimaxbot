/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets PH Sensor
 *  Copyright (c) 2011, Bob Mottram
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
#include "phidgets/phsensor_params.h"

// Handle
CPhidgetPHSensorHandle phid;

// PH sensor state publisher
ros::Publisher phsensor_pub;

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
    ROS_INFO("%s Serial number %d detached!", name, serial_number);

    return 0;
}

int ErrorHandler(CPhidgetHandle phid, void *userptr,
				 int ErrorCode, const char *Description)
{
    ROS_INFO("Error handled. %d - %s", ErrorCode, Description);
    return 0;
}

int PHChangeHandler(CPhidgetPHSensorHandle PH, void *usrptr,
					double Value)
{
    if (initialised) {
        double potential;
        CPhidgetPHSensor_getPotential(PH, &potential);

        phidgets::phsensor_params ph;
        ph.value = (float)Value;
        ph.potential = (float)potential;
        phsensor_pub.publish(ph);

        ROS_INFO("PH: %f -- Potential: %f\n", Value, potential);
    }
    return 0;
}

int display_properties(CPhidgetPHSensorHandle phid)
{
    int serialNo, version;
    double trigger, potential;
    double min, max;
    const char* ptr;

    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

    CPhidgetPHSensor_getPHChangeTrigger(phid, &trigger);
    CPhidgetPHSensor_getPotential(phid, &potential);

    ROS_INFO("%s\n", ptr);
    ROS_INFO("Serial Number: %10d\nVersion: %8d\n",
			 serialNo, version);
    ROS_INFO("Trigger Sensitivity: %f\nPotential: %f\n",
			 trigger, potential);
    CPhidgetPHSensor_getPHMax(phid, &max);
    CPhidgetPHSensor_getPHMin(phid, &min);
    ROS_INFO("Max PH: %0.4lf, Min PH: %0.4lf\n", max, min);
    CPhidgetPHSensor_getPotentialMax(phid, &max);
    CPhidgetPHSensor_getPotentialMin(phid, &min);
    ROS_INFO("Max Potential: %0.2lfmV, " \
			 "Min Potential: %0.2lfmV\n", max, min);

    return 0;
}

bool attach(CPhidgetPHSensorHandle &phid,
			int serial_number)
{
    int result;
    const char *err;

    // create the PH Sensor object
    CPhidgetPHSensor_create(&phid);

    // Set the handlers to be run when the device is plugged
	// in or opened from software, unplugged or closed from
	//  software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid,
								  AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid,
								  DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)phid,
								 ErrorHandler, NULL);

    // Registers a callback that will run if the PH changes
	// by more than the PH trigger.
    // Requires the handle for the PHSensor, the function
	//  that will be called, and a arbitrary pointer that
	//  will be supplied to the callback function (may be NULL).
    CPhidgetPHSensor_set_OnPHChange_Handler(phid,
											PHChangeHandler,
											NULL);

    // open the PH Sensor for device connections
    CPhidget_open((CPhidgetHandle)phid, -1);

    // get the program to wait for an PH Sensor device
	//  to be attached
    ROS_INFO("Waiting for PH Sensor to be attached....");
    if ((result =
		 CPhidget_waitForAttachment((CPhidgetHandle)phid,
									10000))) {
        CPhidget_getErrorDescription(result, &err);
        ROS_INFO("Problem waiting for PH Sensor attachment: %s\n",
				 err);
        return false;
    }
    else return true;
}

/*!
 * \brief disconnect the accelerometer
 */
void disconnect(CPhidgetPHSensorHandle &phid)
{
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_phsensor");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    int serial_number = -1;
    nh.getParam("serial", serial_number);
    std::string name = "phsensor";
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
        if (serial_number > -1) {
            char ser[10];            
            sprintf(ser,"%d", serial_number);
            topic_name += "/";
            topic_name += ser;
        }
        phsensor_pub =
			n.advertise<phidgets::phsensor_params>(topic_name,
												   buffer_length);
        ros::Rate loop_rate(frequency);

        initialised = true;

        while (ros::ok())
			{
				ros::spinOnce();
				loop_rate.sleep();
			}

        disconnect(phid);
    }
    return 0;
}

