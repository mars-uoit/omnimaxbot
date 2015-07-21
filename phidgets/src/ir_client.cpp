/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Example ROS publication to Phidgets IR
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

#include <stdio.h>
#include <ros/ros.h>
#include "phidgets/ir_params.h"


int main(int argc, char** argv)
{
    ROS_INFO("Phidgets IR client");
    ros::init(argc, argv, "ir_client");
    ros::NodeHandle n;

    int serial_number = -1;
    n.getParam("serial", serial_number);
    std::string name = "ir";
    n.getParam("name", name);
    if (serial_number==-1) {
        n.getParam("serial_number", serial_number);
    }

    const int buffer_length = 100;        
    std::string topic_name = "phidgets/" + name;
    if (serial_number > -1) {
        char ser[10];            
        sprintf(ser,"%d", serial_number);
        topic_name += "/";
        topic_name += ser;
    }
    // publish text to the display
    ros::Publisher ir_pub =
		n.advertise<phidgets::ir_params>(topic_name,
										 buffer_length);

    std::string text="";
    ros::Rate loop_rate(30);

    phidgets::ir_params ir;

    //Apple volume up
    ir.code.push_back(0x77);
    ir.code.push_back(0xe1);
    ir.code.push_back(0xd0);
    ir.code.push_back(0xf0);

    //Apple volume up
    int rawData[67] = {
		9040,   4590,    540,    630,    550,
		1740,    550,   1750,    550,   1740,
        550,    620,    550,   1750,    550,
		1740,    550,   1750,    550,   1740,
        550,   1740,    560,   1740,    540,
		630,    550,    620,    550,    620,
        540,    630,    550,   1750,    550,
		1740,    560,   1740,    550,    620,
        550,   1740,    550,    620,    550,
		620,    560,    610,    550,    620,
        550,   1750,    550,   1740,    550,
		620,    550,   1740,    550,   1750,
        550,    620,    550,    620,    550,
		620,    540
    };
    for (int i = 0; i < 67; i++) {
        ir.data.push_back(rawData[i]);
    }

    ir.bitcount = 32;
    ir.head.push_back(9000);
    ir.head.push_back(4500); 
    ir.zero.push_back(560);
    ir.zero.push_back(560);
    ir.one.push_back(560);
    ir.one.push_back(1700);
    ir.trail = 560;
    ir.gap = 110000;
    ir.repeat.push_back(9000);
    ir.repeat.push_back(2250);
    ir.repeat.push_back(560);
    ir.duty_cycle = 33;
    ir.carrier_frequency = 38000;

    ROS_INFO("Hit M to send a test signal");

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        char ch = getc(stdin);

        if ((ch=='m') || (ch=='M')) {

            ROS_INFO("Sending signal");
            ir_pub.publish(ir);
        }
    }

    return 0;
}

