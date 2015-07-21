/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets RFID sensor
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
#include "phidgets/rfid_params.h"

// Handle
CPhidgetRFIDHandle phid;

// RFID state publisher
ros::Publisher rfid_pub;

// RFID message
phidgets::rfid_params r;

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

int OutputChangeHandler(CPhidgetRFIDHandle phid, void *usrptr,
						int Index, int State)
{
    if ((initialised) && ((Index == 0) || (Index == 1))) {
        r.gained = false;
        r.tag="";
        rfid_pub.publish(r);
        ROS_INFO("Index %d State %d", Index, State);
    }
    return 0;
}

int CCONV TagHandler(CPhidgetRFIDHandle phid, void *usrptr,
					 char *TagVal, CPhidgetRFID_Protocol proto)
{
    if (initialised) {
        //turn on the Onboard LED
        CPhidgetRFID_setLEDOn(phid, 1);
    
        r.gained = true;
        char str[5];
        r.tag="";
        for (int i = 0; i < 5; i++) {
            sprintf((char*)str,"%02x",TagVal[i]);
            r.tag += str;
        }
        rfid_pub.publish(r);

        ROS_INFO("Tag Read: %02x%02x%02x%02x%02x\n",
				 TagVal[0], TagVal[1], TagVal[2],
				 TagVal[3], TagVal[4]);
    }
    return 0;
}

int CCONV TagLostHandler(CPhidgetRFIDHandle phid,
						 void *usrptr, char *TagVal,
						 CPhidgetRFID_Protocol proto)
{
    if (initialised) {
        //turn off the Onboard LED
        CPhidgetRFID_setLEDOn(phid, 0);

        r.gained = false;
        rfid_pub.publish(r);

        ROS_INFO("Tag Lost: %02x%02x%02x%02x%02x\n",
				 TagVal[0], TagVal[1], TagVal[2],
				 TagVal[3], TagVal[4]);
    }
    return 0;
}

int display_properties(CPhidgetRFIDHandle phid)
{
    int serialNo, version, numOutputs, antennaOn, LEDOn;
    const char* ptr;

    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

    CPhidgetRFID_getOutputCount (phid, &numOutputs);
    CPhidgetRFID_getAntennaOn (phid, &antennaOn);
    CPhidgetRFID_getLEDOn (phid, &LEDOn);

    ROS_INFO("%s\n", ptr);
    ROS_INFO("Serial Number: %10d\nVersion: %8d\n",
			 serialNo, version);
    ROS_INFO("# Outputs: %d\n\n", numOutputs);
    ROS_INFO("Antenna Status: %d\nOnboard LED Status: %d\n",
			 antennaOn, LEDOn);

    return 0;
}

bool attach(
			CPhidgetRFIDHandle &phid,
			int serial_number)
{
    int result;
    const char *err;

    //Declare an RFID handle
    CPhidgetRFIDHandle rfid = 0;

    //create the RFID object
    CPhidgetRFID_create(&rfid);

    // Set the handlers to be run when the device is plugged
	// in or opened from software, unplugged or closed from
	//  software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)rfid,
								  AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)rfid,
								  DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)rfid,
								 ErrorHandler, NULL);

    // Registers a callback that will run if an output changes.
    // Requires the handle for the Phidget, the function that
	// will be called, and an arbitrary pointer that will be
	//  supplied to the callback function (may be NULL).
    CPhidgetRFID_set_OnOutputChange_Handler(rfid,
											OutputChangeHandler,
											NULL);

    // Registers a callback that will run when a Tag is read.
    // Requires the handle for the PhidgetRFID, the function
	// that will be called, and an arbitrary pointer that
	//  will be supplied to the callback function (may be NULL).
    CPhidgetRFID_set_OnTag2_Handler(rfid, TagHandler, NULL);

    // Registers a callback that will run when a Tag is
	// lost (removed from antenna read range).
    // Requires the handle for the PhidgetRFID, the function
	// that will be called, and an arbitrary pointer that
	//  will be supplied to the callback function (may be NULL).
    CPhidgetRFID_set_OnTagLost2_Handler(rfid,
										TagLostHandler, NULL);

    //open the RFID for device connections
    CPhidget_open((CPhidgetHandle)rfid, -1);

    //get the program to wait for an RFID device to be attached
    ROS_INFO("Waiting for RFID sensor to be attached....");
    if ((result =
		 CPhidget_waitForAttachment((CPhidgetHandle)rfid,
									10000))) {
        CPhidget_getErrorDescription(result, &err);
        ROS_INFO("Problem waiting for RFID sensor " \
				 "attachment: %s\n", err);
        return false;
    }
    else return true;
}

/*!
 * \brief disconnect the RFID reader
 */
void disconnect(
				CPhidgetRFIDHandle &phid)
{
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_rfid");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    int serial_number = -1;
    nh.getParam("serial", serial_number);
    std::string name = "rfid";
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
        rfid_pub =
			n.advertise<phidgets::rfid_params>(topic_name,
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

