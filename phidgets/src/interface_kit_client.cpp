/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Example ROS subscription to Phidgets interface kit
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

ros::ServiceClient client_interface_kit;

/*!
 * \brief callback when the sensor or output values change
 * \param ptr sensor parameters
 */
void interfaceKitCallback(const phidgets::interface_kit_params::ConstPtr& ptr)
{
    phidgets::interface_kit_params ifk = *ptr;
    switch(ifk.value_type) {
	case 1: { // digital input
		ROS_INFO("Digital input %d State %d",
				 ifk.index, ifk.value);
		break;
	}
	case 2: { // digital output
		ROS_INFO("Digital output %d State %d",
				 ifk.index, ifk.value);
		break;
	}
	case 3: { // sensor
		ROS_INFO("Sensor %d Value %d",
				 ifk.index, ifk.value);
		break;
	}
	}
}

/*!
 * \brief set the state of a digital output
 * \param output_index index of the digital output
 * \param state state of the digital output (0 or 1)
 */
void set_digital_output(int output_index,
						int state)
{
	phidgets::interface_kit srv;
	srv.request.index = output_index;
	srv.request.value_type = 1;
	srv.request.value = state;
	srv.response.ack = 0;
	if (client_interface_kit.call(srv)) {
		if ((int)srv.response.ack == 1) {
			ROS_INFO("Changed digital output %d to state %d",
					 output_index, state);
		}
		else {
			ROS_INFO("Returned %d", (int)srv.response.ack);
		}
	}
	else {
		ROS_ERROR("Failed to call service interface_kit");
	}
}

/*!
 * \brief set a sensor trigger level
 * \param sensor_index index of the sensor
 * \param trigger trigger level
 */
void set_sensor_trigger_level(int sensor_index,
							  int trigger_level)
{
	phidgets::interface_kit srv;
	srv.request.index = sensor_index;
	srv.request.value_type = 2;
	srv.request.value = trigger_level;
	srv.response.ack = 0;
	if (client_interface_kit.call(srv)) {
		if ((int)srv.response.ack == 1) {
			ROS_INFO("Changed sensor %d trigger level %d",
					 sensor_index, trigger_level);
		}
		else {
			ROS_INFO("Returned %d", (int)srv.response.ack);
		}
	}
	else {
		ROS_ERROR("Failed to call service interface_kit");
	}
}

int main(int argc, char** argv)
{
	ROS_INFO("Phidgets Interface Kit client");
	ros::init(argc, argv, "interface_kit_client");
	ros::NodeHandle n;
	ros::Subscriber interface_kit_sub =
		n.subscribe("phidgets/interface_kit", 1,
					interfaceKitCallback);
	client_interface_kit =
		n.serviceClient<phidgets::interface_kit>("interface_kit");

	set_digital_output(0, 0);
	ros::spin();
	return 0;
}

