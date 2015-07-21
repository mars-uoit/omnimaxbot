/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets manager
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
#include "phidgets/manager_params.h"

// Handle
CPhidgetManagerHandle phid;

// manager state publisher
ros::Publisher manager_pub;

bool initialised = false;

int AttachHandler(CPhidgetHandle phid, void *userptr)
{
    int serialNo;
    const char *name;
    CPhidget_DeviceID id;
    CPhidget_DeviceClass cls;

    CPhidget_getDeviceName (phid, &name);
    CPhidget_getSerialNumber(phid, &serialNo);
    CPhidget_getDeviceClass(phid, &cls);
    CPhidget_getDeviceID(phid, &id);

    phidgets::manager_params m;
    m.attached = true;
    m.name.data = name;
    m.serial = serialNo;
    m.device_class = cls;
    m.device_id = id;
    if (initialised) manager_pub.publish(m);
    ROS_INFO("Attached device %s Serial " \
			 "number %d Class %d ID %d",
			 name, serialNo, cls, id);

    return 0;
}

int DetachHandler(CPhidgetHandle phid, void *userptr)
{
    int serialNo;
    const char *name;

    CPhidget_getDeviceName (phid, &name);
    CPhidget_getSerialNumber(phid, &serialNo);

    phidgets::manager_params m;
    m.attached = false;
    m.name.data = name;
    m.serial = serialNo;
    if (initialised) manager_pub.publish(m);
    ROS_INFO("Removed device %s Serial number %d",
			 name, serialNo);

    return 0;
}

int ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr,
				 int ErrorCode, const char *Description)
{
    ROS_INFO("Error handled. %d - %s", ErrorCode, Description);
    return 0;
}

int display_devices(CPhidgetManagerHandle phid)
{
    int serialNo, version, numDevices, i;
    const char* ptr;
    CPhidgetHandle *devices;

    CPhidgetManager_getAttachedDevices (phid, &devices,
										&numDevices);

    printf("|-   # -|-              Type" \
		   "              -|- Serial No. -|-  Version -|\n");
    printf("|-------|--------------------" \
		   "--------------|--------------|------------|\n");


    for(i = 0; i < numDevices; i++)	{
		CPhidget_getDeviceType(devices[i], &ptr);
		CPhidget_getSerialNumber(devices[i], &serialNo);
		CPhidget_getDeviceVersion(devices[i], &version);

		printf("|- %3d -|- %30s -|- %10d -|- %8d -|\n",
			   i, ptr, serialNo, version);
		printf("|-------|-----------------------------" \
			   "-----|--------------|------------|\n");
	}

    CPhidgetManager_freeAttachedDevicesArray(devices);

    return 0;
}

bool attach(CPhidgetManagerHandle &phid)
{
    // create the object
    CPhidgetManager_create(&phid);

    // Set the handlers to be run when the device is
	// plugged in or opened from software, unplugged
	// or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid,
								  AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid,
								  DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)phid,
								 ErrorHandler, NULL);

    CPhidgetManager_open(phid);
    return true;
}

/*!
 * \brief disconnect the encoder
 */
void disconnect(CPhidgetManagerHandle &phid)
{
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_manager");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    int frequency = 10;
    nh.getParam("frequency", frequency);

    if (attach(phid)) {

        const int buffer_length = 100;        
        std::string topic_name = "phidgets/manager";
        manager_pub =
			n.advertise<phidgets::manager_params>(topic_name,
												  buffer_length);
        ros::Rate loop_rate(frequency);

        initialised = true;

		display_devices(phid);

        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        disconnect(phid);
    }
    return 0;
}

