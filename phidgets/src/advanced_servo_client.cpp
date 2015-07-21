/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Example ROS subscription to Phidgets advanced servo controller
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
#include "phidgets/servo_reference.h"
#include "phidgets/servo_params.h"

#define PHIDGETS_ADVANCED_SERVO_MOTORS 8

ros::ServiceClient client_servo_reference;

/*!
 * \brief callback when a servo has changed position
 * \param servo parameters
 */
void servoCallback(const phidgets::servo_params::ConstPtr& ptr)
{
	phidgets::servo_params s = *ptr;
	ROS_INFO("Servo %d position %.2f", s.index, s.position);
}

/*!
 * \brief set the position, speed and acceleration for a servo
 * \param index servo index
 * \param engage whether to energise the servo
 * \param position reference position for the servo
 * \param speed reference speed for the servo
 * \param acceleration reference acceleration for the servo
 */
void set_servo_reference(
						 int index,
						 bool engage,
						 float position,
						 float speed,
						 float acceleration)
{
	phidgets::servo_reference srv;
	srv.request.index = index;
	srv.request.engage = engage;
	srv.request.position = position;
	srv.request.speed = speed;
	srv.request.acceleration = acceleration;
	srv.response.ack = 0;
	if (client_servo_reference.call(srv)) {
		if ((int)srv.response.ack == 1) {
			ROS_INFO("Changed servo %d reference %.2f",
					 index, position);
		}
		else {
			ROS_INFO("Returned %d", (int)srv.response.ack);
		}
	}
	else {
		ROS_ERROR("Failed to call service servo_reference");
	}
}

/*!
 * \brief energises all servos and sends them
 *        to their neutral position
 * \param neutral_position servo position
 * \param speed servo speed
 * \param acceleration servo acceleration
 */
void energise(
			  float neutral_position,
			  float speed,
			  float acceleration)
{
    for (int servo_index = 0;
		 servo_index < PHIDGETS_ADVANCED_SERVO_MOTORS;
		 servo_index++) {
        set_servo_reference(servo_index, true,
							neutral_position, speed,
							acceleration);
    }
}

/*!
 * \brief deactivates all servos
 */
void disengage()
{
    for (int servo_index = 0;
		 servo_index < PHIDGETS_ADVANCED_SERVO_MOTORS;
		 servo_index++) {
        set_servo_reference(servo_index, false, 0, 0, 0);
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "advanced_servo_client");
	ros::NodeHandle n;
	ros::Subscriber servo_sub =
		n.subscribe("phidgets/servos", 1, servoCallback);
	client_servo_reference =
		n.serviceClient<phidgets::servo_reference>("servo_reference");

	ros::Rate loop_rate(30);
	while(ros::ok) {    
		set_servo_reference(1, true, 255/2 - 23, 20,20);
		set_servo_reference(2, true, 255/2 - 23, 20,20);
		ros::spinOnce();
		loop_rate.sleep();

		if ((int)getc(stdin) == 27) break;
	}
	disengage();

	return 0;
}

