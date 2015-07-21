/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets IR remote
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
#include "phidgets/ir_params.h"

// Handle
CPhidgetIRHandle phid;

// Text LCD state subscriber
ros::Subscriber ir_sub;

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

int CodeHandler(CPhidgetIRHandle ir, void *userPtr,
				unsigned char *data, int dataLength,
				int bitCount, int repeat)
{
    int i;
    ROS_INFO("DataLength: %d, Bit Count: %d, Repeat: %d\n",
			 dataLength, bitCount, repeat);
    ROS_INFO("Code: ");
    for (i = 0; i < dataLength; i++) {
        ROS_INFO("%02x", data[i]); 
    }
    return 0;
}

int LearnHandler(CPhidgetIRHandle phid, void *userPtr,
				 unsigned char *data, int dataLength,
				 CPhidgetIR_CodeInfoHandle codeInfo)
{
    int i;
    ROS_INFO("Learned Code: ");
    for(i = 0; i < dataLength; i++) {
        printf("%02x", data[i]);
    }

    std::string encodingStr = "";
    switch(codeInfo->encoding) {
	case PHIDGET_IR_ENCODING_UNKNOWN:
		encodingStr = "Unknown";
		break;
	case PHIDGET_IR_ENCODING_SPACE:
		encodingStr = "Space";
		break;
	case PHIDGET_IR_ENCODING_PULSE:
		encodingStr = "Pulse";
		break;
	case PHIDGET_IR_ENCODING_BIPHASE:
		encodingStr = "BiPhase";
		break;
	case PHIDGET_IR_ENCODING_RC5:
		encodingStr = "RC5";
		break;
	case PHIDGET_IR_ENCODING_RC6:
		encodingStr = "RC6";
		break;
	default:
		encodingStr = "Unknown";
		break;
    }
    std::string lengthStr = "";
    switch(codeInfo->length) {
	case PHIDGET_IR_LENGTH_UNKNOWN:
		lengthStr = "Unknown";
		break;
	case PHIDGET_IR_LENGTH_CONSTANT:
		lengthStr = "Constant";
		break;
	case PHIDGET_IR_LENGTH_VARIABLE:
		lengthStr = "Variable";
		break;
	default:
		lengthStr = "Unknown";
		break;
    }

    printf("Learned Code Info:\n");
    printf("----------------------------------------------------\n");
	
    printf("Bit Count: %d\nEncoding: %s\nLength: %s\nGap: %d\nTrail: %d",
		   codeInfo->bitCount, encodingStr.c_str(),
		   lengthStr.c_str(), codeInfo->gap, codeInfo->trail);
    printf("Header: { %d, %d }\nOne: { %d, %d }\nZero: { %d, %d }\n",
		   codeInfo->header[0], codeInfo->header[1],
		   codeInfo->one[0], codeInfo->one[1],
		   codeInfo->zero[0], codeInfo->zero[1]);
    printf("Repeat: {");
    for(i = 0; i < 26; i++) {
        if(codeInfo->repeat[i] == 0) break;
        if (i == 0) {
            printf("%d", codeInfo->repeat[i]);
        }
        else {
            printf(", %d", codeInfo->repeat[i]);
        }
    }
    printf("}\n");
    printf("MinRepeat: %d\n", codeInfo->min_repeat);
    printf("Toggle Mask: ");

    int toggleMaskBytes = 0;
    if ((codeInfo->bitCount % 8) == 0) {
        toggleMaskBytes = (codeInfo->bitCount / 8) + 0;
    }
    else {
        toggleMaskBytes = (codeInfo->bitCount / 8) + 1;
    }

    for (i = 0; i < toggleMaskBytes; i++) {
        printf("%02x", codeInfo->toggle_mask[i]);
    }
    printf("\n");
    printf("Carrier Frequency: %d\nDuty Cycle: %d\n",
		   codeInfo->carrierFrequency, codeInfo->dutyCycle);
    printf("----------------------------------------------------\n");

    return 0;
}

int RawDataHandler(CPhidgetIRHandle phid, void *userPtr,
				   int *data, int dataLength)
{
    int i;
    printf("----------------------------------------------------\n");
    for(i = 0; i < dataLength; i++) {
        if ((i % 8) == 0) printf("\n");
        if (data[i] == 0x7fffffff)
            printf("LONG");
        else
            printf("%d", data[i]);
        if (((i + 1) % 8) != 0) printf(", ");
    }
    printf("\n----------------------------------------------------\n");
    return 0;
}

int display_properties(CPhidgetIRHandle phid)
{
    int serialNo, version;
    const char* ptr;

    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

    ROS_INFO("%s\n", ptr);
    ROS_INFO("Serial Number: %10d\nVersion: %8d\n",
			 serialNo, version);

    return 0;
}

bool attach(CPhidgetIRHandle &phid,
			int serial_number)
{
    int result;
    const char *err;

    CPhidgetIR_create(&phid);

    CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid,
								  AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid,
								  DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)phid,
								 ErrorHandler, NULL);

    CPhidgetIR_set_OnCode_Handler(phid, CodeHandler, NULL);
    CPhidgetIR_set_OnLearn_Handler(phid, LearnHandler, NULL);
    CPhidgetIR_set_OnRawData_Handler(phid, RawDataHandler, NULL);

    CPhidget_open((CPhidgetHandle)phid, -1);

    ROS_INFO("Waiting for PhidgetIR to be attached.... \n");
    if ((result =
		 CPhidget_waitForAttachment((CPhidgetHandle)phid,
									10000))) {
        CPhidget_getErrorDescription(result, &err);
        ROS_INFO("Problem waiting for attachment: %s\n", err);
        return false;
    }
    else {
        return true;
    }
}

/*!
 * \brief disconnect the IR display
 */
void disconnect(CPhidgetIRHandle &phid)
{
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

/*!
 * \brief callback when new IR is requested
 * \param ptr IR parameters
 */
void irCallback(const phidgets::ir_params::ConstPtr& ptr)
{
    phidgets::ir_params ir = *ptr;
    int result;
    const char *err;

    //This is standard NEC code
    CPhidgetIR_CodeInfo codeInfo;
    codeInfo.encoding = PHIDGET_IR_ENCODING_SPACE;
    codeInfo.bitCount = ir.bitcount;
    codeInfo.header[0] =
		ir.head[0], codeInfo.header[1] = ir.head[1];
    codeInfo.zero[0] =
		ir.zero[0], codeInfo.zero[1] = ir.zero[1];
    codeInfo.one[0] = ir.one[0], codeInfo.one[1] = ir.one[1];
    codeInfo.trail = ir.trail;
    codeInfo.gap = ir.gap;
    codeInfo.repeat[0] =
		ir.repeat[0], codeInfo.repeat[1] =
		ir.repeat[1], codeInfo.repeat[2] = ir.repeat[2];

    std::string code_str = "";
    for (int i = 0; i < (int)ir.code.size(); i++) {
        code_str += (char)ir.code[i];
    }
    if ((result =
		 CPhidgetIR_Transmit(phid,
							 (unsigned char*)(code_str.c_str()),
							 &codeInfo))) {
        CPhidget_getErrorDescription(result, &err);
        ROS_INFO("IR Error: %s", err);
    }
    else {
        if (ir.data.size() > 0) {
            int *dat = new int[(int)ir.data.size()];
            for (int i = 0; i < (int)ir.data.size(); i++) {
                dat[i] = ir.data[i];
            }
            CPhidgetIR_TransmitRaw(phid, dat,
								   (int)ir.data.size(),
								   ir.carrier_frequency,
								   ir.duty_cycle, ir.gap);
            delete[] dat;
        }
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_ir");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    int serial_number = -1;
    nh.getParam("serial", serial_number);
    std::string name = "ir";
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
        ros::Subscriber ir_sub = n.subscribe(topic_name, 1,
											 irCallback);
        ros::Rate loop_rate(frequency);

        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        disconnect(phid);
    }
    return 0;
}

