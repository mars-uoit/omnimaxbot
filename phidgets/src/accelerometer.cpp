/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets accelerometer
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
#include "phidgets/accelerometer_params.h"

// Handle
CPhidgetAccelerometerHandle phid;

// accelerometer state publisher
ros::Publisher accelerometer_pub;

// acceleration values
phidgets::accelerometer_params acc;

// axis indices
int axis_id[3];

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
    ROS_INFO("Error handled. %d - %s",
			 ErrorCode, Description);
    return 0;
}

// update tilt angles in degrees
void update_tilt()
{
    acc.orientation.clear();

    // tilt
    double ratio,angle=0;
    angle = asin(acc.acceleration[axis_id[1]])*180/3.1415927;
    acc.orientation.push_back(angle);

    if (acc.acceleration.size() > 2) {
        // roll
        angle =
			asin(acc.acceleration[axis_id[0]])*180/3.1415927;
        acc.orientation.push_back(angle);
    }
}

// callback that will run if the acceleration
// changes by more than the Acceleration trigger.
// Index - Index of the axis that is generating 
// the event, Value - the value read by the accelerometer axis
int accel_AccelChangeHandler(CPhidgetAccelerometerHandle WGT,
							 void *userptr, int Index,
							 double Value)
{
    if (initialised) {
        acc.acceleration[axis_id[Index]] = Value;
        acc.stamp[axis_id[Index]] = ros::Time::now();
        update_tilt();
        accelerometer_pub.publish(acc);
    }
    ROS_INFO("Axis %d Value %.8f", Index, (float)Value);
    return 0;
}

int display_properties(CPhidgetAccelerometerHandle phid)
{
    int serial_number, version, num_axes;
    const char* ptr;

    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid,
							 &serial_number);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid,
							  &version);

    CPhidgetAccelerometer_getAxisCount((CPhidgetAccelerometerHandle)phid,
									   &num_axes);

    // initialise acceleration values
    ros::Time begin = ros::Time::now();
    acc.acceleration.clear();
    acc.stamp.clear();
    for (int i = 0; i < num_axes; i++) {
        acc.acceleration.push_back(0);
        acc.stamp.push_back(begin);
        acc.stamp.push_back(begin);
        acc.stamp.push_back(begin);
    }

    ROS_INFO("%s", ptr);
    ROS_INFO("Serial Number: %d", serial_number);
    ROS_INFO("Version: %d", version);
    ROS_INFO("Number of axes %d", num_axes);

    return 0;
}

bool attach(
			CPhidgetAccelerometerHandle &phid,
			int serial_number,
			double sensitivity)
{
    // create the object
    CPhidgetAccelerometer_create(&phid);

    // Set the handlers to be run when the device is
	// plugged in or opened from software, unplugged or
	// closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid,
								  AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid,
								  DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)phid,
								 ErrorHandler, NULL);

    // Registers a callback that will run if the
	// acceleration changes by more than the Acceleration
	// trigger.
    // Requires the handle for the Accelerometer, the
	// function that will be called, 
    // and an arbitrary pointer that will be supplied to
	// the callback function (may be NULL)
    CPhidgetAccelerometer_set_OnAccelerationChange_Handler(phid,
														   accel_AccelChangeHandler,
														   NULL);

    //open the device for connections
    CPhidget_open((CPhidgetHandle)phid, serial_number);

    // get the program to wait for an accelerometer
	// device to be attached
    if (serial_number == -1) {
        ROS_INFO("Waiting for Accelerometer Phidget to " \
				 "be attached....");
    }
    else {
        ROS_INFO("Waiting for Accelerometer Phidget %d " \
				 "to be attached....", serial_number);
    }
    int result;
    if((result =
		CPhidget_waitForAttachment((CPhidgetHandle)phid,
								   10000)))	{
			const char *err;
			CPhidget_getErrorDescription(result, &err);
			ROS_ERROR("Problem waiting for attachment: %s",
					  err);
			return false;
		}
    else {
		// get the number of available axes on the
		// attached accelerometer
        int numAxes=2;
		CPhidgetAccelerometer_getAxisCount(phid, &numAxes);

        // set the sensitivity
		for (int i = 0; i < numAxes; i++) {
			CPhidgetAccelerometer_setAccelerationChangeTrigger(phid,
															   i,
															   sensitivity);
        }
        return true;
    }
}

/*!
 * \brief disconnect the accelerometer
 */
void disconnect(CPhidgetAccelerometerHandle &phid)
{
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_accelerometer");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    int serial_number = -1;
    nh.getParam("serial", serial_number);
    std::string name = "accelerometer";
    nh.getParam("name", name);
    if (serial_number==-1) {
        nh.getParam("serial_number", serial_number);
    }

    std::string topic_path = "phidgets/";
    nh.getParam("topic_path", topic_path);

    double sensitivity=0.0;
    nh.getParam("sensitivity", sensitivity);

    int frequency = 30;
    nh.getParam("frequency", frequency);

    int ax_id = 0;
    nh.getParam("x_axis_id", ax_id);
    axis_id[0] = ax_id;

    ax_id = 1;
    nh.getParam("y_axis_id", ax_id);
    axis_id[1] = ax_id;

    ax_id = 2;
    nh.getParam("z_axis_id", ax_id);
    axis_id[2] = ax_id;

    if (attach(phid, serial_number, sensitivity)) {
		display_properties(phid);

        const int buffer_length = 100;        
        std::string topic_name = topic_path + name;
        if (serial_number > -1) {
            char ser[10];            
            sprintf(ser,"%d", serial_number);
            topic_name += "/";
            topic_name += ser;
        }
        accelerometer_pub =
			n.advertise<phidgets::accelerometer_params>(topic_name,
														buffer_length);
        ros::Rate loop_rate(frequency);

        initialised = true;

        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        disconnect(phid);
    }
    return 0;
}
