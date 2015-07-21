/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Example ROS subscription to Phidgets high speed encoder
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
#include "phidgets/encoder_params.h"

/*!
 * \brief callback when the encoder count changes
 * \param ptr encoder parameters
 */
void encoderCallback(const phidgets::encoder_params::ConstPtr& ptr)
{
    phidgets::encoder_params e = *ptr;
    ROS_INFO("Encoder %d Count %d", e.index, e.count);
}

int main(int argc, char** argv)
{
	ROS_INFO("Phidgets High Speed Encoder client");
	ros::init(argc, argv, "high_speed_encoder_client");
	ros::NodeHandle n;
	ros::Subscriber encoder_sub =
		n.subscribe("phidgets/encoder", 1, encoderCallback);
	ros::spin();
	return 0;
}

