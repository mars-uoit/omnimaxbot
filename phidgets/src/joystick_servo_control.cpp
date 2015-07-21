/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Joystick control of Phidgets motor control HC
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
#include "phidgets/joystick_params.h"

ros::ServiceClient client_servo_reference;
ros::ServiceClient client_joystick;

float speed = 20;
float acceleration = 50;
float deadband = 0.01f;

// index numbers of the servos to be controlled
int servo_index0 = 0;
int servo_index1 = 1;

bool initialised = false;

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
 * \brief joystick command has arrived
 */
void joystickCallback(const phidgets::joystick_params::ConstPtr& ptr)
{
    if (initialised) {
        // is joystick servo control enabled ?
        bool enable_servo_control = false;
        ros::NodeHandle n;
        n.getParam("joystick/enable_servo_control",
				   enable_servo_control);
        if (enable_servo_control) {
            // get the joystick values
            phidgets::joystick_params joy = *ptr;
            float rotate = joy.axes[0];
            float forward = joy.axes[1];
            bool button_press = false;
            if (joy.buttons[0] != 0) button_press = true;

            ROS_INFO("Joystick %f %f", rotate, forward);
            ROS_INFO("Deadband %f", deadband);
            ROS_INFO("Speed %f  Accel %f", speed, acceleration);

            if ((fabs(rotate) > deadband) || 
                (fabs(forward) > deadband)) {
                // joystick moved
                set_servo_reference(servo_index0, true,
									127 + (rotate*127),
									speed, acceleration);
                set_servo_reference(servo_index1, true,
									127 + (forward*127),
									speed, acceleration);

            }
            else {
                // joystick centred
                set_servo_reference(servo_index0, true, 127,
									speed, acceleration);
                set_servo_reference(servo_index1, true, 127,
									speed, acceleration);
            }
        }
    }
}

int main(int argc, char** argv)
{
    ROS_INFO("Joystick control of Phidgets " \
			 "advanced servo controller");
    ros::init(argc, argv, "joystick_servo_control");
    ros::NodeHandle n;

    ros::NodeHandle nh("~");
    std::string servo_reference_name = "servo_reference";
    nh.getParam("servoreference", servo_reference_name);
    std::string joystick_name = "joystick";
    nh.getParam("joystickname", joystick_name);
    std::string servo_controller_name = "servos";
    nh.getParam("servocontrollername", servo_controller_name);
    std::string topic_path = "phidgets/";
    nh.getParam("topic_path", topic_path);
    bool enable = false;
    nh.getParam("enable", enable);
    int v;
    nh.getParam("speed", v);
    if ((v > 1) && (v < 100)) speed = (float)v;
    nh.getParam("acceleration", v);
    if ((v > 1) && (v < 100)) acceleration = (float)v;
    nh.getParam("deadband", v);
    if ((v > 0.02) && (v <= 1.0)) deadband = (float)v;
    nh.getParam("servoindex0", servo_index0);
    nh.getParam("servoindex1", servo_index1);
    if (servo_index0 == servo_index1) {
        servo_index0 = 0;
        servo_index1 = 1;
    }

    ros::Subscriber servo_sub =
		n.subscribe(topic_path + servo_controller_name,
					1, servoCallback);
    ros::Subscriber joystick_sub =
		n.subscribe(topic_path + joystick_name, 1,
					joystickCallback);

    sleep(3); // pause required by joystick_servo_control.launch
    client_servo_reference =
		n.serviceClient<phidgets::servo_reference>(servo_reference_name);

    // dissable the servos
    set_servo_reference(servo_index0, false, 127, speed,
						acceleration);
    set_servo_reference(servo_index1, false, 127, speed,
						acceleration);

    // make the joystick active
    n.setParam("joystick/enable", true);

    n.setParam("joystick/enable_servo_control", enable);

    if (!enable) {
        ROS_INFO("Waiting for enable parameter to be set");
    }

    int frequency = 30;
    n.setParam("frequency", frequency);

    ros::Rate loop_rate(frequency);
    initialised = true;
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    set_servo_reference(servo_index0, false, 127, speed,
						acceleration);
    set_servo_reference(servo_index1, false, 127, speed,
						acceleration);

    return 0;
}

