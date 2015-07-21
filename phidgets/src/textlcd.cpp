/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets Text LCD Display
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
#include "phidgets/textlcd_params.h"

// Handle
CPhidgetTextLCDHandle phid;

// Text LCD state subscriber
ros::Subscriber rfid_sub;

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

int display_properties(CPhidgetTextLCDHandle phid)
{
    int serialNo, version, numRows, numColumns;
	int backlight, cursor, contrast, cursor_blink;
    const char* ptr;

    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

    CPhidgetTextLCD_getRowCount (phid, &numRows);
    CPhidgetTextLCD_getColumnCount (phid, &numColumns);
    CPhidgetTextLCD_getBacklight (phid, &backlight);
    CPhidgetTextLCD_getContrast (phid, &contrast);
    CPhidgetTextLCD_getCursorOn (phid, &cursor);
    CPhidgetTextLCD_getCursorBlink (phid, &cursor_blink);

    ROS_INFO("%s\n", ptr);
    ROS_INFO("Serial Number: %10d\nVersion: %8d\n",
			 serialNo, version);
    ROS_INFO("# Rows: %d\n# Columns: %d\n",
			 numRows, numColumns);
    ROS_INFO("Current Contrast Level: %d\nBacklight Status: %d\n",
			 contrast, backlight);
    ROS_INFO("Cursor Status: %d\nCursor Blink Status: %d\n",
			 cursor, cursor_blink);

    return 0;
}

bool attach(
			CPhidgetTextLCDHandle &phid,
			int serial_number)
{
    int result;
    const char *err;

    // create the TextLCD object
    CPhidgetTextLCD_create(&phid);

    // Set the handlers to be run when the device is plugged
	// in or opened from software, unplugged or closed from
	// software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid,
								  AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid,
								  DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)phid,
								 ErrorHandler, NULL);

    // open the TextLCD for device connections
    CPhidget_open((CPhidgetHandle)phid, -1);

    // get the program to wait for an TextLCD device to be
	// attached
    ROS_INFO("Waiting for Text LCD to be attached....");
    if ((result =
		 CPhidget_waitForAttachment((CPhidgetHandle)phid,
									10000))) {
        CPhidget_getErrorDescription(result, &err);
        ROS_INFO("Problem waiting for Text LCD attachment: %s\n",
				 err);
        return false;
    }
    else return true;
}

/*!
 * \brief disconnect the Text LCD display
 */
void disconnect(
				CPhidgetTextLCDHandle &phid)
{
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

/*!
 * \brief callback when new text is requested
 * \param ptr Text LCD parameters
 */
void textlcdCallback(const phidgets::textlcd_params::ConstPtr& ptr)
{
    phidgets::textlcd_params lcd = *ptr;
    for (int i = 0; i < (int)lcd.row.size(); i++) {
        CPhidgetTextLCD_setDisplayString (phid, i,
										  (char*)lcd.row[i].c_str());
    }
    CPhidgetTextLCD_setContrast(phid, lcd.contrast);
    if (lcd.cursor) {
        CPhidgetTextLCD_setCursorOn(phid, 1);
    }
    else {
        CPhidgetTextLCD_setCursorOn(phid, 0);
    }
    if (lcd.blink) {
        CPhidgetTextLCD_setCursorBlink(phid, 1);
    }
    else {
        CPhidgetTextLCD_setCursorBlink(phid, 0);
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_textlcd");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    int serial_number = -1;
    nh.getParam("serial", serial_number);
    std::string name = "textlcd";
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

        std::string topic_name = topic_path + name;
        if (serial_number > -1) {
            char ser[10];            
            sprintf(ser,"%d", serial_number);
            topic_name += "/";
            topic_name += ser;
        }
        ros::Subscriber rfid_sub =
			n.subscribe(topic_name, 1, textlcdCallback);
        ros::Rate loop_rate(frequency);

        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        disconnect(phid);
    }
    return 0;
}

