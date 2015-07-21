/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Example ROS subscription to Phidgets RFID sensor
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
#include "phidgets/rfid_params.h"

/*!
 * \brief callback when the RFID values change
 * \param ptr RFID parameters
 */
void rfidCallback(const phidgets::rfid_params::ConstPtr& ptr)
{
    phidgets::rfid_params r = *ptr;
    if (r.gained) {
        ROS_INFO("Tag read: %s",r.tag.c_str());
    }
    else {
        ROS_INFO("Tag lost: %s",r.tag.c_str());
    }
}

int main(int argc, char** argv)
{
	ROS_INFO("Phidgets RFID client");
	ros::init(argc, argv, "rfid_client");
	ros::NodeHandle n;
	ros::Subscriber rfid_sub =
		n.subscribe("phidgets/rfid", 1, rfidCallback);
	ros::spin();
	return 0;
}

