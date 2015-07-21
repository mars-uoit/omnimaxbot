/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Example ROS publication to Phidgets LED
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
#include "phidgets/led_params.h"



int main(int argc, char** argv)
{
    const int no_of_LEDs = 64;

    ROS_INFO("Phidgets LED client");
    ros::init(argc, argv, "led_client");
    ros::NodeHandle n;

    int serial_number = -1;
    n.getParam("serial", serial_number);
    std::string name = "led";
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
    ros::Publisher led_pub =
		n.advertise<phidgets::led_params>(topic_name,
										  buffer_length);

    std::string text="";
    ros::Rate loop_rate(30);

    phidgets::led_params led;
    led.brightness.clear();
    for (int i = 0; i < no_of_LEDs; i++) {
        led.brightness.push_back(0);
    }

    ROS_INFO("Press keys to illuminate LEDs");

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        char ch = getc(stdin);
        int index = -1;
        if ((ch>='a') && (ch<='z')) {
            index = (int)(ch - 'a');
        }
        if ((ch>='A') && (ch<='Z')) {
            index = (int)(ch - 'A');
        }
        if ((index >= 0) &&
			(index < (int)led.brightness.size())) {
            led.brightness[index] = 100 - led.brightness[index];
            ROS_INFO("Sending text");
            led_pub.publish(led);
        }
    }

    return 0;
}

