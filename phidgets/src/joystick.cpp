/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Analog joystick plugged into Phidgets interface kit
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

#include <stdio.h>
#include <ros/ros.h>
#include "phidgets/interface_kit_params.h"
#include "phidgets/interface_kit.h"
#include "phidgets/joystick_params.h"

bool initialised = false;
phidgets::joystick_params params;
ros::Publisher joystick_pub;
int horizontal_axis = 1;
int vertical_axis = 2;
int button_input = -1;

int horizontal_direction = 1;
int vertical_direction = 1;

/*!
 * \brief callback when the sensor or output values change
 * \param ptr sensor parameters
 */
void interfaceKitCallback(const phidgets::interface_kit_params::ConstPtr& ptr)
{
    if (initialised) {
        ros::NodeHandle n;
        bool enable = false;
        n.getParam("joystick/enable", enable);
        
        phidgets::interface_kit_params ifk = *ptr;
        switch(ifk.value_type)
			{
            case 1: { // digital input
                if (ifk.index == button_input) {
                    if (ifk.value != 0) {
                        params.buttons[0] = 1;
                        ROS_INFO("Button pressed");
                        if (enable) {
							joystick_pub.publish(params);
						}
                    }
                    else {
                        params.buttons[0] = 0;
                    }
                }
                break;
            }
            case 2: { // digital output
                ROS_INFO("Digital output %d State %d",
						 ifk.index, ifk.value);
                break;
            }
            case 3: { // sensor
                if (ifk.index == horizontal_axis) {
                    params.axes[0] =
						(float)((ifk.value-500)/500.0f) *
						horizontal_direction;
                    ROS_INFO("Horizontal %.2f", params.axes[0]);
                    if (enable) joystick_pub.publish(params);
                }
                else {
                    if (ifk.index == vertical_axis) {
                        params.axes[1] =
							(float)((ifk.value-500)/500.0f) *
							vertical_direction;
                        ROS_INFO("Vertical %.2f",
								 params.axes[1]);
                        if (enable) {
							joystick_pub.publish(params);
						}
                    }
                }
            
                break;
            }
			}
    }
}

int main(int argc, char** argv)
{
	ROS_INFO("Phidgets Joysick");
	ros::init(argc, argv, "joystick");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	n.setParam("joystick/enable", false);
	std::string name = "joystick";
	nh.getParam("name", name);
	std::string interface_kit_name = "interface_kit";
	nh.getParam("interfacekitname", interface_kit_name);
	std::string topic_path = "phidgets/";
	nh.getParam("topic_path", topic_path);

	params.axes.clear();
	params.axes.push_back(0.0f);
	params.axes.push_back(0.0f);
	params.buttons.clear();
	params.buttons.push_back(0);

	ros::Subscriber interface_kit_sub =
		n.subscribe(topic_path + interface_kit_name, 1,
					interfaceKitCallback);
	joystick_pub =
		n.advertise<phidgets::joystick_params>(topic_path +
											   name, 50);

	int v;
	nh.getParam("horizontalaxis", horizontal_axis);
	nh.getParam("verticalaxis", vertical_axis);
	nh.getParam("horizontaldirection", v);
	if (v != 0) horizontal_direction = v;
	nh.getParam("verticalaxis", vertical_axis);
	nh.getParam("verticaldirection", v);
	if (v != 0) vertical_direction = v;
	nh.getParam("buttoninput", button_input);

	int frequency = 30;
	nh.getParam("frequency", frequency);

	if (horizontal_direction != -1) horizontal_direction = 1;
	if (vertical_direction != -1) vertical_direction = 1;

	initialised = true;

	ros::Rate update_rate(frequency);
	while(ros::ok()){
		ros::spinOnce();
		update_rate.sleep();
	}
	return 0;
}

