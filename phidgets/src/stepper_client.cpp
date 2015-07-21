/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Example ROS subscription to Phidgets stepper motor control
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
#include "phidgets/stepper_params.h"

ros::Publisher pub;

/*!
 * \brief callback when the stepper motor position changes
 * \param ptr motor parameters
 */
void stepperCallback(const phidgets::stepper_params::ConstPtr& ptr)
{
    phidgets::stepper_params m = *ptr;
    ROS_INFO("Motor %d Position %lld", m.index, m.position);
}

/*!
 * \brief set the speed and acceleration of a motor
 * \param motor_index index of the motor
 * \param engage whether to energise the motor
 * \param velocity velocity of the motor
 * \param acceleration acceleration of the motor
 * \param position target position
 * \param reset_position whether to reset the position count
 */
void set_motor_reference(
						 int motor_index,
						 bool engage,
						 float velocity,
						 float acceleration,
						 long long position,
						 bool reset_position)
{
    phidgets::stepper_params s;
    s.index = motor_index;
    s.engage = engage;
    s.velocity = velocity;
    s.acceleration = acceleration;
    s.position = position;
    s.reset_position = reset_position;
    pub.publish(s);
}

/*!
 * \brief sets all positions to zero
 * \param no_of_stepper_motors number of stepper motors on the board
 */
void zero_stepper_posiions(int no_of_stepper_motors)
{
    for (int motor_index = 0; motor_index < no_of_stepper_motors; motor_index++) {
        set_motor_reference(motor_index, false, 0,0,0, true);
    }
}

int main(int argc, char** argv)
{
    ROS_INFO("Phidgets Stepper Motor Control client");
    ros::init(argc, argv, "stepper_client");
    ros::NodeHandle n;
    ros::Subscriber stepper_sub =
		n.subscribe("phidgets/stepper", 1, stepperCallback);
    pub = n.advertise<phidgets::stepper_params>("stepper", 10);

    // wait for the server node to start up
    sleep(3);

    zero_stepper_posiions(4);

    ros::Rate loop_rate(30);
    while(ros::ok) {
        set_motor_reference(0, true, 20, 20, 100, false);
        set_motor_reference(1, true, 20, 20, 100, false);
  
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

