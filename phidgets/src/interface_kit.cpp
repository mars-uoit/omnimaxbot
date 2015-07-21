/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets interface kit
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
#include "phidgets/interface_kit_params.h"
#include "phidgets/interface_kit.h"

// handle
CPhidgetInterfaceKitHandle phid;

// interface kit state publisher
ros::Publisher interface_kit_pub;

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

// callback that will run if an input changes.
// Index - Index of the input that generated the event,
// State - boolean (0 or 1) representing the input
// state (on or off)
int InputChangeHandler(CPhidgetInterfaceKitHandle IFK,
					   void *usrptr, int Index, int State)
{
    phidgets::interface_kit_params m;
    m.index = Index;
    m.value_type = 1;
    m.value = State;
    if (initialised) interface_kit_pub.publish(m);
    ROS_INFO("Digital input %d State %d", Index, State);
    return 0;
}

// callback that will run if an output changes.
// Index - Index of the output that generated the event,
// State - boolean (0 or 1) representing the output
// state (on or off)
int OutputChangeHandler(CPhidgetInterfaceKitHandle IFK,
						void *usrptr, int Index, int State)
{
    phidgets::interface_kit_params m;
    m.index = Index;
    m.value_type = 2;
    m.value = State;
    if (initialised) interface_kit_pub.publish(m);
    ROS_INFO("Digital output %d State %d", Index, State);
    return 0;
}

// callback that will run if the sensor value changes by
// more than the OnSensorChange trigger.
// Index - Index of the sensor that generated the event,
// Value - the sensor read value
int SensorChangeHandler(CPhidgetInterfaceKitHandle IFK,
						void *usrptr, int Index, int Value)
{
    phidgets::interface_kit_params m;
    m.index = Index;
    m.value_type = 3;
    m.value = Value;
    if (initialised) interface_kit_pub.publish(m);
    ROS_INFO("Sensor %d Value %d", Index, Value);
    return 0;
}

int display_properties(CPhidgetInterfaceKitHandle phid)
{
    int serial_number, version, ratiometric;
	int num_sensors, num_inputs, num_outputs, triggerVal;
    const char* ptr;

    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid,
							 &serial_number);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

    CPhidgetInterfaceKit_getInputCount(phid, &num_inputs);
    CPhidgetInterfaceKit_getOutputCount(phid, &num_outputs);
    CPhidgetInterfaceKit_getSensorCount(phid, &num_sensors);
    CPhidgetInterfaceKit_getRatiometric(phid, &ratiometric);

    ROS_INFO("%s", ptr);
    ROS_INFO("Serial Number: %d", serial_number);
    ROS_INFO("Version: %d", version);
    ROS_INFO("Number of digital inputs %d", num_inputs);
    ROS_INFO("Number of digital outputs %d", num_outputs);
    ROS_INFO("Number of sensors %d", num_sensors);
    ROS_INFO("Ratiometric %d", ratiometric);

    for (int i = 0; i < num_sensors; i++) {       
		CPhidgetInterfaceKit_getSensorChangeTrigger (phid,
													 i,
													 &triggerVal);
		//CPhidgetInterfaceKit_setSensorChangeTrigger (phid, i, 10);
		ROS_INFO("Sensor %d Sensitivity Trigger %d",
				 i, triggerVal);
	}

    return 0;
}

bool attach(CPhidgetInterfaceKitHandle &phid,
			int serial_number)
{
    CPhidget_enableLogging(PHIDGET_LOG_VERBOSE,
						   "/home/motters/Desktop/phidgets.log");

    // create the object
    CPhidgetInterfaceKit_create(&phid);

    // Set the handlers to be run when the device is plugged
	// in or opened from software, unplugged or closed from
	// software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid,
								  AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid,
								  DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)phid,
								 ErrorHandler, NULL);

    // Registers a callback that will run if an input changes.
    // Requires the handle for the Phidget, the function that
	// will be called, and an arbitrary pointer that will
	// be supplied to the callback function (may be NULL).
    CPhidgetInterfaceKit_set_OnInputChange_Handler (phid,
													InputChangeHandler,
													NULL);

    // Registers a callback that will run if the sensor
	// value changes by more than the OnSensorChange trig-ger.
    // Requires the handle for the IntefaceKit, the
	// function that will be called, and an arbitrary
	// pointer that will be supplied to the callback
	// function (may be NULL).
    CPhidgetInterfaceKit_set_OnSensorChange_Handler (phid,
													 SensorChangeHandler,
													 NULL);

    // Registers a callback that will run if an output changes.
    // Requires the handle for the Phidget, the function
	// that will be called, and an arbitrary pointer that
	// will be supplied to the callback function (may be NULL).
    CPhidgetInterfaceKit_set_OnOutputChange_Handler (phid,
													 OutputChangeHandler,
													 NULL);

    //open the device for connections
    CPhidget_open((CPhidgetHandle)phid, serial_number);

    // get the program to wait for an interface kit device
	// to be attached
    if (serial_number == -1) {
        ROS_INFO("Waiting for Interface Kit Phidget " \
				 "to be attached....");
    }
    else {
        ROS_INFO("Waiting for Interface Kit Phidget %d " \
				 "to be attached....", serial_number);
    }
    int result;
    if((result =
		CPhidget_waitForAttachment((CPhidgetHandle)phid,
								   10000))) {
			const char *err;
			CPhidget_getErrorDescription(result, &err);
			ROS_ERROR("Problem waiting for attachment: %s",
					  err);
			return false;
		}
    else return true;
}

/*!
 * \brief disconnect the interface kit
 */
void disconnect(CPhidgetInterfaceKitHandle &phid)
{
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

/*!
 * \brief set digital output state or sensor trigger level
 * \param req requested parameters
 * \param res returned parameters
 */
bool set_values(phidgets::interface_kit::Request &req,
				phidgets::interface_kit::Response &res)
{
    switch(req.value_type) {
	case 1: { // set digital output
		CPhidgetInterfaceKit_setOutputState (phid, req.index,
											 req.value);
		ROS_INFO("Output %d State %d", req.index, req.value);
		break;
	}
	case 2: { // set sensor trigger level
		CPhidgetInterfaceKit_setSensorChangeTrigger(phid,
													req.index,
													req.value);
		ROS_INFO("Sensor %d Trigger level %d",
				 req.index, req.value);
		break;
	}
	}

    res.ack = 1;

    return(true);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_interface_kit");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    int serial_number = -1;
    nh.getParam("serial", serial_number);
    std::string name = "interface_kit";
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
        std::string service_name = "interface_kit";
        if (serial_number > -1) {
            char ser[10];
            sprintf(ser,"%d", serial_number);
            topic_name += "/";
            topic_name += ser;
            service_name += "/";
            service_name += ser;
        }
        interface_kit_pub =
			n.advertise<phidgets::interface_kit_params>(topic_name,
														buffer_length);

        // start service which can be used to set
		// digital outputs
        ros::ServiceServer service =
			n.advertiseService(service_name, set_values);

        initialised = true;
        ros::Rate loop_rate(frequency);

        while (ros::ok())
			{
				ros::spinOnce();
				loop_rate.sleep();
			}
    }
    return 0;
}

