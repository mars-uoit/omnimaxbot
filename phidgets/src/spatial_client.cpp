/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Example ROS subscription to Phidgets spatial
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
#include "phidgets/spatial_params.h"

/*!
 * \brief callback when the spatial values change
 * \param ptr spatial parameters
 */
void spatialCallback(const phidgets::spatial_params::ConstPtr& ptr)
{
    phidgets::spatial_params s = *ptr;
    int count = (int)s.acceleration.size()/3;
    for (int i = 0; i < count; i++) {
        ROS_INFO("=== Data Set: %d ===\n", i);
        ROS_INFO("Acceleration %6f %6f %6f",
				 s.acceleration[i*3], s.acceleration[i*3+1],
				 s.acceleration[i*3+2]);
        ROS_INFO("Tilt/Roll %6f %6f",
				 s.orientation[i*2], s.orientation[i*2+1]);
        ROS_INFO("Angular      %6f %6f %6f",
				 s.angular[i*3], s.angular[i*3+1],
				 s.angular[i*3+2]);
        ROS_INFO("Magnetic     %6f %6f %6f",
				 s.magnetic[i*3], s.magnetic[i*3+1],
				 s.magnetic[i*3+2]);
        ROS_INFO("Timestamp    %6f", s.timestamp[i]);
        ROS_INFO("-----------------------------------------------");
    }
}

int main(int argc, char** argv)
{
	ROS_INFO("Phidgets Spatial client");
	ros::init(argc, argv, "spatial_client");
	ros::NodeHandle n;
	ros::Subscriber spatial_sub =
		n.subscribe("phidgets/spatial", 1, spatialCallback);
	ros::spin();
	return 0;
}

