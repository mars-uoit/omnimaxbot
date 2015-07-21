/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Example ROS subscription to Phidgets motor control HC
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
#include <geometry_msgs/Twist.h>
#include "phidgets/motor_params.h"

ros::Publisher motor_pub;

/*!
 * \brief callback when the motor parameters change
 * \param ptr motor parameters
 */
void motorCallback(const phidgets::motor_params::ConstPtr& ptr)
{
    phidgets::motor_params m = *ptr;
    switch(m.value_type)
		{
        case 1: { // inputs
            ROS_INFO("Motor input %d State %.2f",
					 m.index, m.value);
            break;
        }
        case 2: { // velocity
            ROS_INFO("Motor %d Velocity %.2f",
					 m.index, m.value);
            break;
        }
        case 3: { // current
            ROS_INFO("Motor %d Current %.2f",
					 m.index, m.value);
            break;
        }
		}    
}

/*!
 * \brief sets all velocities to zero
 */
void stop_motors()
{
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    motor_pub.publish(cmd);
    ROS_INFO("Motors stopped");
}

int main(int argc, char** argv)
{
    ROS_INFO("Phidgets Motor Control HC client");
    ros::init(argc, argv, "motor_control_hc_client");
    ros::NodeHandle n;
    ros::Subscriber motor_control_sub =
		n.subscribe("phidgets/motorcontrol", 1, motorCallback);

    // publish motor commands in geometry_msgs::Twist format
    motor_pub =
		n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    ros::Rate loop_rate(10);
    geometry_msgs::Twist motor_command;
    while(n.ok()) {
        // vector values should be in the range -1 <= x <= 1
        motor_command.linear.x = 0.0f;
        motor_command.linear.y = 0.5f;
        motor_command.linear.z = 0.0f;
        motor_pub.publish(motor_command);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

