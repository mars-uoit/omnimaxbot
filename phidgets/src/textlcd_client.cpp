/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Example ROS publication to Phidgets Text LCD
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
#include "phidgets/textlcd_params.h"



int main(int argc, char** argv)
{
    ROS_INFO("Phidgets Text LCD client");
    ros::init(argc, argv, "textlcd_client");
    ros::NodeHandle n;

    int serial_number = -1;
    n.getParam("serial", serial_number);
    std::string name = "textlcd";
    n.getParam("name", name);
    if (serial_number==-1) {
        n.getParam("serial_number", serial_number);
    }
    std::string topic_path = "phidgets/";
    n.getParam("topic_path", topic_path);

    const int buffer_length = 100;        
    std::string topic_name = topic_path + name;
    if (serial_number > -1) {
        char ser[10];            
        sprintf(ser,"%d", serial_number);
        topic_name += "/";
        topic_name += ser;
    }
    // publish text to the display
    ros::Publisher textlcd_pub =
		n.advertise<phidgets::textlcd_params>(topic_name,
											  buffer_length);

    std::string text="";
    ros::Rate loop_rate(30);

    phidgets::textlcd_params text_message;
    text_message.row.clear();
    text_message.row.push_back("");
    text_message.row.push_back("");
    text_message.contrast = 127;
    text_message.cursor = false;
    text_message.blink = false;
    int row_index = 0;

    ROS_INFO("Type some text and press Enter");

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        char ch = getc(stdin);

        if (ch!=10) {
            text += ch;
        }
        else {
            text_message.row[row_index]=text;
            row_index = 1 - row_index;
            ROS_INFO("Sending text");
            textlcd_pub.publish(text_message);
            text="";
        }
    }

    return 0;
}

