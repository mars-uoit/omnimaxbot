/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Glimpse using pan and tilt servos
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
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "phidgets/servo_reference.h"
#include "phidgets/servo_params.h"

ros::ServiceClient client_servo_reference;

float speed = 20;
float acceleration = 50;

// index numbers of the servos to be controlled
int servo_index0 = 0;
int servo_index1 = 1;

ros::Publisher joint_pub;
std::string pan_joint="",tilt_joint="";

int **position;
int **angle_degrees;

bool initialised = false;

sensor_msgs::JointState joint_state;

// converts an angle in degrees into a position
// in servo coordinates
int get_position(int target_angle_degrees,
				 int **position, int **angle_degrees,
				 int index)
{
    int dangle =
		angle_degrees[index][1] - angle_degrees[index][0];
    int dpos = position[index][1] - position[index][0];
    int pos =
		position[index][0] +
		((target_angle_degrees - angle_degrees[index][0]) *
		 dpos / dangle);
    if ((pos < 0) || (pos > 255)) {
        pos = 127;
        ROS_INFO("Position out of range %d", pos);
    }
    return pos;
}

// converts a position in servo coordinates into an
// angle in radians
float get_angle(int current_position, int **position,
				int **angle_degrees, int index)
{
    int dangle =
		angle_degrees[index][1] - angle_degrees[index][0];
    float dpos = position[index][1] - position[index][0];

    float angle =
		(angle_degrees[index][0] +
		 ((current_position - position[index][0]) *
		  dangle / dpos))* 3.1415927/180;
    return angle;
}

/*!
 * \brief callback when a servo has changed position
 * \param servo parameters
 */
void servoCallback(const phidgets::servo_params::ConstPtr& ptr)
{
    if (initialised) {
        phidgets::servo_params s = *ptr;

        if (s.index == servo_index0) {
            joint_state.position[0] =
				get_angle(s.position, position,
						  angle_degrees, 0);
            ROS_INFO("%s = %.2f",
					 joint_state.name[0].c_str(),
					 joint_state.position[0]);
        }
        else {
            joint_state.position[1] =
				get_angle(s.position, position,
						  angle_degrees, 1);
            ROS_INFO("%s = %.2f",
					 joint_state.name[1].c_str(),
					 joint_state.position[1]);
        }
    }
}

/*!
 * \brief set the position, speed and acceleration for a servo
 * \param index servo index
 * \param engage whether to energise the servo
 * \param position reference position for the servo
 * \param speed reference speed for the servo
 * \param acceleration reference acceleration for the servo
 */
void set_servo_reference(int index,
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

bool file_exists(char * filename)
{
    FILE * fp = fopen(filename,"rb");
    if (fp != NULL) {
        fclose(fp);
        return true;
    }
    return false;
}

int main(int argc, char** argv)
{
    ROS_INFO("Gather point cloud using pan and tilt servos");
    ros::init(argc, argv, "servo_glimpse");
    ros::NodeHandle n;

    int pan_index=0, tilt_index=0,no_of_cameras=1;
    int min_pan_angle_degrees, max_pan_angle_degrees;
    int min_tilt_angle_degrees, max_tilt_angle_degrees;
    position = new int*[2];
    angle_degrees = new int*[2];

    joint_pub =
		n.advertise<sensor_msgs::JointState>("joint_states", 1);

    ros::NodeHandle nh("~");
    std::string glimpse_command="",final_command="";
	std::string directory="";
    std::string glimpse_command2="",final_command2="";

    // Joint identifiers within a URDF model
    pan_joint="";
    tilt_joint="";
    std::string urdf_filename="";
    nh.getParam("panjoint", pan_joint);
    nh.getParam("tiltjoint", tilt_joint);
    //urdf::Model model;
    //nh.getParam("urdf", urdf_filename);

    // amount of time in seconds to pause at the
	// end whilst servos
    // return to their neutral position
    int end_time_sec = 3;
    nh.getParam("endtime", end_time_sec);

    // parse the URDF model
    //if ((urdf_filename != "") &&
	//    (pan_joint != "") && (tilt_joint != "")) {
    //    if (!model.initFile(urdf_filename)) {
    //        ROS_ERROR("Failed to parse urdf file");
    //        return -1;
    //    }
    //    ROS_INFO("Successfully parsed urdf file");
    //}

    nh.getParam("glimpsecommand", glimpse_command);
    nh.getParam("finalcommand", final_command);
    nh.getParam("glimpsecommand2", glimpse_command2);
    nh.getParam("finalcommand2", final_command2);
    nh.getParam("outputdirectory", directory);

    std::string topic_path = "phidgets/";
    nh.getParam("topic_path", topic_path);

    if (glimpse_command2 != "") {
        no_of_cameras = 2;
    }

    if (glimpse_command == "") {
        ROS_WARN("No glimpse command specified");
    }

    for (int i = 0; i < 3; i++) {
        position[i] = new int[2];
        angle_degrees[i] = new int[2];
    }

    bool enable = false;
    nh.getParam("enable", enable);
    int v,pos,retval,forward=0,vertical=0,baseline=0;
    nh.getParam("forward", forward);
    nh.getParam("vertical", vertical);
    nh.getParam("baseline", baseline);
    nh.getParam("speed", v);
    if ((v > 1) && (v < 256)) speed = (float)v;
    nh.getParam("acceleration", v);
    if ((v > 1) && (v < 256)) acceleration = (float)v;

    nh.getParam("servo0pos0", pos);
    position[0][0] = pos;
    nh.getParam("servo0angle0", pos);
    angle_degrees[0][0] = pos;
    nh.getParam("servo0pos1", pos);
    position[0][1] = pos;
    nh.getParam("servo0angle1", pos);
    angle_degrees[0][1] = pos;

    nh.getParam("servo1pos0", pos);
    position[1][0] = pos;
    nh.getParam("servo1angle0", pos);
    angle_degrees[1][0] = pos;
    nh.getParam("servo1pos1", pos);
    position[1][1] = pos;
    nh.getParam("servo1angle1", pos);
    angle_degrees[1][1] = pos;

    nh.getParam("servoindex0", servo_index0);
    nh.getParam("servoindex1", servo_index1);
    if (servo_index0 == servo_index1) {
        servo_index0 = 0;
        servo_index1 = 1;
    }

    nh.getParam("minpanangle", min_pan_angle_degrees);
    nh.getParam("maxpanangle", max_pan_angle_degrees);

    nh.getParam("mintiltangle", min_tilt_angle_degrees);
    nh.getParam("maxtiltangle", max_tilt_angle_degrees);

    int pan_steps=3, tilt_steps=3;
	int glimpse_time_sec=2, direction=1,ctr=0;
    nh.getParam("pansteps", pan_steps);
    nh.getParam("tiltsteps", tilt_steps);
    nh.getParam("glimpsetime", glimpse_time_sec);

    std::string servo_reference_name = "servo_reference";
    nh.getParam("servoreference", servo_reference_name);

    std::string servo_controller_name = "servos";
    nh.getParam("servocontrollername", servo_controller_name);

    int frequency = 30;
    nh.getParam("frequency", frequency);

    ros::Subscriber servo_sub =
		n.subscribe(topic_path + servo_controller_name,
					1, servoCallback);

    sleep(3); // pause
    client_servo_reference =
		n.serviceClient<phidgets::servo_reference>(servo_reference_name);

    // dissable the servos
    set_servo_reference(servo_index0,
						false, 127, speed, acceleration);
    set_servo_reference(servo_index1, false, 127,
						speed, acceleration);

    sleep(3); // pause

    // enable the servos
    set_servo_reference(servo_index0, true, 127, speed,
						acceleration);
    set_servo_reference(servo_index1, true, 127, speed,
						acceleration);

    bool valid = true;
    if (angle_degrees[0][0] == angle_degrees[0][1]) {
        ROS_INFO("No variance in pan angle\n");
        valid = false;
    }
    if (angle_degrees[1][0] == angle_degrees[1][1]) {
        ROS_INFO("No variance in tilt angle\n");
        valid = false;
    }

    if (position[0][0] == position[0][1]) {
        ROS_INFO("No variance in pan position\n");
        valid = false;
    }
    if (position[1][0] == position[1][1]) {
        ROS_INFO("No variance in tilt position\n");
        valid = false;
    }

    int camera_translation[3];
    camera_translation[0] = 0;
    camera_translation[1] = 0;
    camera_translation[2] = forward;

    int pan_degrees = angle_degrees[0][0];
    int tilt_degrees = min_tilt_angle_degrees;

    //if (pan_steps > 1) pan_degrees = min_pan_angle_degrees + (pan_index * (max_pan_angle_degrees - min_pan_angle_degrees) / (pan_steps-1));
    if (tilt_steps > 1) {
        tilt_degrees = min_tilt_angle_degrees +
			(tilt_index *
			 (max_tilt_angle_degrees - min_tilt_angle_degrees) /
			 (tilt_steps-1));
    }
    //pos = get_position(pan_degrees, position, angle_degrees, 0);
    //set_servo_reference(servo_index0, true, pos, speed, acceleration);
    pos = get_position(tilt_degrees, position,
					   angle_degrees, 1);
    set_servo_reference(servo_index1, true, pos,
						speed, acceleration);
    //sleep(3);

    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.name[0] = pan_joint;
    joint_state.name[1] = tilt_joint;

    ros::Time begin = ros::Time::now();
    ros::Rate loop_rate(frequency);
    initialised = true;
    char str[2000];
    std::string point_cloud_files="";
    point_cloud_files += '"';

    sprintf((char*)str,"rm %sgl*.dat", directory.c_str());
    retval = system(str);

    pan_index=pan_steps/2-1;
    while((ros::ok()) && (valid)) {
        ros::spinOnce();
        loop_rate.sleep();

        joint_state.header.stamp = ros::Time::now();
        joint_pub.publish(joint_state);

        double elapsed_sec = (ros::Time::now() - begin).toSec();
        if (elapsed_sec >= glimpse_time_sec) {
            begin = ros::Time::now();

            if (pan_steps > 1) {
                pan_degrees =
					min_pan_angle_degrees +
					(pan_index *
					 (max_pan_angle_degrees -
					  min_pan_angle_degrees) / (pan_steps-1));
            }
            else {
                pan_degrees = min_pan_angle_degrees;
            }
            pos = get_position(pan_degrees, position,
							   angle_degrees, 0);
            set_servo_reference(servo_index0, true, pos,
								speed, acceleration);

            sleep(glimpse_time_sec/(no_of_cameras*2));

            pan_index--;
            if (ctr >= (pan_steps/2)-1) break;
            ctr++;
        }
    }

    pan_index=tilt_index=0;
    ctr=0;
    valid=true;
    while((ros::ok()) && (valid)) {
        ros::spinOnce();
        loop_rate.sleep();

        joint_state.header.stamp = ros::Time::now();
        joint_pub.publish(joint_state);

        double elapsed_sec = (ros::Time::now() - begin).toSec();
        if (elapsed_sec >= glimpse_time_sec) {
            begin = ros::Time::now();

            if (pan_steps > 1) {
				pan_degrees =
					min_pan_angle_degrees +
					(pan_index * (max_pan_angle_degrees -
								  min_pan_angle_degrees) /
					 (pan_steps-1));
			}
            if (tilt_steps > 1) {
                tilt_degrees =
					min_tilt_angle_degrees +
					(tilt_index * (max_tilt_angle_degrees -
								   min_tilt_angle_degrees) /
					 (tilt_steps-1));
            }

            pos = get_position(pan_degrees, position,
							   angle_degrees, 0);
            set_servo_reference(servo_index0, true, pos,
								speed, acceleration);
            pos = get_position(tilt_degrees, position,
							   angle_degrees, 1);
            set_servo_reference(servo_index1, true, pos,
								speed, acceleration);

            for (int cam = 0; cam < no_of_cameras; cam++) {
                int pan = pan_degrees + (cam*180);
                int tilt = tilt_degrees;

                std::string cmd = glimpse_command;
                if (cam == 1) {
                    cmd = glimpse_command2;
                }

                if (vertical != 0) {
                    camera_translation[2] =
						(int)(vertical *
							  sin(tilt*3.1415927/180.0));
                    camera_translation[1] =
						(int)(vertical *
							  cos(tilt*3.1415927/180.0));
                    forward = camera_translation[2];
                } 

                double pan_radians = pan*3.1415927/180.0;
                camera_translation[0] =
					(int)((-baseline*0.5*cos(pan_radians)) -
						  (forward * sin(pan_radians)));
                camera_translation[2] =
					(int)((-baseline*0.5*sin(pan_radians)) +
						  (forward * cos(pan_radians)));

                if (cmd != "") {
                    sprintf((char*)str,cmd.c_str(),
							tilt, pan,
							camera_translation[0],
							camera_translation[1],
							camera_translation[2],
							directory.c_str(),
							ctr + (cam*pan_steps*tilt_steps));
                    ROS_INFO((char*)str);
                    sleep(glimpse_time_sec/(no_of_cameras*2));
                    retval = system(str);
                }

                sleep(glimpse_time_sec/(no_of_cameras*2));

                if (cmd != "") {
                    if (cam == 0) {
                        sprintf((char*)str,"%sgl%d.dat",
								directory.c_str(), ctr);
                        if (ctr > 0) point_cloud_files+=",";
                    }
                    else {
                        sprintf((char*)str,",%sgl%d.dat",
								directory.c_str(),
								ctr+(cam*pan_steps*tilt_steps));
                    }
                    point_cloud_files += str;
                }
            }

            if (direction == 1) {
                if (pan_index >= pan_steps-1) {
                    tilt_index++;
                    direction = 1 - direction;
                }
                else {
                    pan_index++;
                }
            }
            else {
                if (pan_index <= 0) {
                    tilt_index++;
                    direction = 1 - direction;
                }
                else {
                    pan_index--;
                }
            }

            if (ctr >= pan_steps*tilt_steps-1) break;
            ctr++;
        }
    }

    if (final_command!="") {
        point_cloud_files += '"';
        sprintf((char*)str,final_command.c_str(),
				point_cloud_files.c_str(),directory.c_str());
        ROS_INFO((char*)str);
        retval = system(str);
    }

    // neutral tilt position
    valid = true;
    if (tilt_index > 0) {
        tilt_index--;
    }
    if (tilt_index < 0) {
        tilt_index++;
    }
    while((ros::ok()) && (valid)) {
        ros::spinOnce();
        loop_rate.sleep();

        joint_state.header.stamp = ros::Time::now();
        joint_pub.publish(joint_state);

        double elapsed_sec = (ros::Time::now() - begin).toSec();
        if (elapsed_sec >= glimpse_time_sec) {
            begin = ros::Time::now();

            if (tilt_steps > 1) {
                tilt_degrees =
					min_tilt_angle_degrees +
					(tilt_index * (max_tilt_angle_degrees -
								   min_tilt_angle_degrees) /
					 (tilt_steps-1));
            }
            else {
                tilt_degrees = min_tilt_angle_degrees;
            }
            pos = get_position(tilt_degrees, position,
							   angle_degrees, 1);
            set_servo_reference(servo_index1, true, pos,
								speed, acceleration);

            sleep(glimpse_time_sec/(no_of_cameras*2));

            if (tilt_index == 0) break;
            if (tilt_index > 0) {
                tilt_index--;
            }
            if (tilt_index < 0) {
                tilt_index++;
            }
        }
    }

    // neutral pan position
    valid=true;
    if (pan_index > (pan_steps/2)) {
        pan_index--;
    }
    if (pan_index < (pan_steps/2)) {
        pan_index++;
    }
    while((ros::ok()) && (valid)) {
        ros::spinOnce();
        loop_rate.sleep();

        joint_state.header.stamp = ros::Time::now();
        joint_pub.publish(joint_state);

        double elapsed_sec = (ros::Time::now() - begin).toSec();
        if (elapsed_sec >= glimpse_time_sec) {
            begin = ros::Time::now();

            if (pan_steps > 1) {
                pan_degrees =
					min_pan_angle_degrees +
					(pan_index * (max_pan_angle_degrees -
								  min_pan_angle_degrees) /
					 (pan_steps-1));
            }
            else {
                pan_degrees = min_pan_angle_degrees;
            }
            pos = get_position(pan_degrees, position,
							   angle_degrees, 0);
            set_servo_reference(servo_index0, true, pos,
								speed, acceleration);

            sleep(glimpse_time_sec/(no_of_cameras*2));

            if (pan_index == (pan_steps/2)) break;
            if (pan_index > (pan_steps/2)) {
                pan_index--;
            }
            if (pan_index < (pan_steps/2)) {
                pan_index++;
            }
        }
    }

    set_servo_reference(servo_index0, true, 127, speed,
						acceleration);
    set_servo_reference(servo_index1, true, 127, speed,
						acceleration);

    double t0 = ros::Time::now().toSec();
    double t1 = ros::Time::now().toSec();
    while ((int)(t1-t0) <= end_time_sec) {
        ros::spinOnce();
        loop_rate.sleep();

        t1 = ros::Time::now().toSec();
        joint_state.header.stamp = ros::Time::now();
        joint_pub.publish(joint_state);
    }
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = 0;
    joint_state.position[1] = 0;
    joint_pub.publish(joint_state);

    // dissable the servos
    set_servo_reference(servo_index0, false, 127, speed,
						acceleration);
    set_servo_reference(servo_index1, false, 127, speed,
						acceleration);

    for (int i = 0; i < 2; i++) {
        delete [] position[i];
        delete [] angle_degrees[i];
        position[i] = NULL;
        angle_degrees[i] = NULL;
    }
    delete position;
    delete angle_degrees;

    return 0;
}

