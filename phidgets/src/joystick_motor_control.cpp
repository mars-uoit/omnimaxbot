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

#include <unistd.h>
#include <stdio.h>
#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include "phidgets/motor_params.h"
#include "phidgets/joystick_params.h"
#include "phidgets/interface_kit_params.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::ServiceClient client_joystick;

bool initial_pose_set = false;

// set if joystick control is resumed during
// navigation to a goal
bool joystick_control_resumed = false;

float speed = 20;
float acceleration = 20;
float deadband = 0.1f;

// digital inputs on the interface kit used to
// start and stop joystick control
int start_button_input_index = -1;
int stop_button_input_index = -1;
int learn_button_input_index = -1;
int place_button_input_index = -1;
int set_pose_button_input_index = -1;
int go_button_input_index = -1;

// command to be run when the start or stop button is pressed
std::string start_button_command = "";
std::string stop_button_command = "";
std::string learn_button_command = "";

// speech sounds made when start or stop buttons are pressed
std::string start_button_sound = "Started";
std::string stop_button_sound = "Stopped";
std::string enabled_sound = "Joystick control enabled";
std::string learn_button_sound = "%s remembered";
std::string set_pose_button_sound = "Location set to %s";
std::string go_button_sound = "Going to %s";
std::string arrived_goal_sound = "Arrived at %s";
std::string failed_goal_sound = "I can't get to %s";
std::string joystick_resumed_sound = "Joystick control";

// Topic subscribed to in order to obtain poses,
// typically /acml_pose
std::string pose_topic = "";

// Topic to publish pose array
std::string pose_array_topic = "/particlecloud";

// Topic to publish the initial pose to
std::string initial_pose_topic = "/initialpose";

// File used to store poses
std::string placemarks_filename="places.txt";

// The current pose estimate
geometry_msgs::PoseWithCovarianceStamped current_pose;
bool pose_received = false;

// place names
int current_place_name_index = -1;
std::vector<std::string> place_names;

// Is the robot moving to a goal
bool moving_to_goal = false;
MoveBaseClient *ac = NULL;

// A file containing the current pose information
// This allows the robot to recover its pose after
// being switched off
std::string current_pose_filename="";

// Initial pose translational uncertainty in metres
float initial_pose_radius = 0.1f;

// Initial pose angular uncertainty in radians
float initial_pose_orientation_variance = M_PI/12.0;

ros::Publisher motor_pub;
ros::Publisher initial_pose_pub;
ros::Publisher pose_array_pub;
geometry_msgs::Twist cmd;

double max_linear_velocity = 0.3;
double max_angular_velocity = 20*3.1415927/180.0;

bool started = false;
bool initialised = false;

// What time was the learn button previously pressed ?
ros::Time previous_learn_button_press;

// Which button was previously pressed ?
int previous_button = -1;

/*!
 * \brief has joystick control been enabled
 * \return true if joystick motor control is enabled
 */
bool is_enabled()
{
    bool enable_motor_control = false;
    ros::NodeHandle n;
    n.getParam("joystick/enable_motor_control",
			   enable_motor_control);
    return enable_motor_control;
}

bool valid_pose()
{
    const double max = 99999;
    if ((current_pose.pose.pose.position.x>-max) &&
        (current_pose.pose.pose.position.x<max) &&
        (current_pose.pose.pose.position.y>-max) &&
        (current_pose.pose.pose.position.y<max) &&
        (current_pose.pose.pose.orientation.z>-max) &&
        (current_pose.pose.pose.orientation.z<max) &&
        (current_pose.pose.pose.orientation.w>-max) &&
        (current_pose.pose.pose.orientation.w<max)) {
        return true;
    }
    return false;
}

/*
 * \brief Saves the current pose to a file
 */
void save_current_pose(std::string filename)
{
    if (valid_pose()) {
        FILE * fp = fopen(filename.c_str(),"w");
        if (fp != NULL) {
            ROS_INFO("Saving current pose %.3f, %.3f, %.3f, %.3f",
					 current_pose.pose.pose.position.x,
					 current_pose.pose.pose.position.y,
					 current_pose.pose.pose.orientation.z,
					 current_pose.pose.pose.orientation.w);
            fprintf(fp,"%f,%f,%f,%f",
					current_pose.pose.pose.position.x,
					current_pose.pose.pose.position.y,
					current_pose.pose.pose.orientation.z,
					current_pose.pose.pose.orientation.w);
            fclose(fp);
        }
    }
}

int read_placemarks(std::vector<std::string> &places,
					std:: vector<float> &poses)
{
    int idx=0,index=-1,record_index=0;
    FILE * fp = fopen(placemarks_filename.c_str(), "r");
    if (fp != NULL) {
        std::string str="";
        while (!feof(fp)) {
            char c = fgetc(fp);
            if (c == ',') {
                if (idx == 0) {
                    places.push_back(str);
                    if (current_place_name_index > -1) {
                        if (str==
							place_names[current_place_name_index]) {
                            index = record_index;
                        }
                    }
                }
                else {
                    poses.push_back((float)atof(str.c_str()));
                }
                str="";
                idx++;
            }
            else {
                if (c == '\n') {
                    poses.push_back((float)atof(str.c_str()));
                    idx = 0;
                    record_index++;
                    str="";
                }
                else {
                    str += c;
                }
            }
        }
        if (str!="") {
            poses.push_back((float)atof(str.c_str()));
        }
        fclose(fp);                
    }
    return index;
}

void learn_pose()
{
    if (valid_pose()) {

        if (is_enabled()) {
            sound_play::SoundClient sc;
            if ((pose_received) && (placemarks_filename != "") && (current_place_name_index > -1)) {
                bool saved = false;

                if (learn_button_command != "") {
                    ROS_INFO("Running command:\n%s",learn_button_command.c_str());
                    system(learn_button_command.c_str());
                }

                std::vector<std::string> places;
                std:: vector<float> poses;

                int index = read_placemarks(places, poses);

                // save to file
                if (index == -1) {
                    // append or write new place to file
                    FILE * fp =
						fopen(placemarks_filename.c_str(), "a");
                    if (fp == NULL) {
						fp = fopen(placemarks_filename.c_str(),
								   "w");
					}
                    if (fp != NULL) {
                        ROS_INFO("%s, %.3f, %.3f, %.3f, %.3f",
								 place_names[current_place_name_index].c_str(),
								 current_pose.pose.pose.position.x,
								 current_pose.pose.pose.position.y,
								 current_pose.pose.pose.orientation.z,
								 current_pose.pose.pose.orientation.w);
                        fprintf(fp,"%s, %f, %f, %f, %f\n",
								place_names[current_place_name_index].c_str(),
								current_pose.pose.pose.position.x,
								current_pose.pose.pose.position.y,
								current_pose.pose.pose.orientation.z,
								current_pose.pose.pose.orientation.w);
                        fclose(fp);
                        saved = true;
                    }
                }
                else {
                    // alter an existing place
                    poses[index*4] =
						current_pose.pose.pose.position.x;
                    poses[index*4+1] =
						current_pose.pose.pose.position.y;
                    poses[index*4+2] =
						current_pose.pose.pose.orientation.z;
                    poses[index*4+3] =
						current_pose.pose.pose.orientation.w;
                    FILE * fp =
						fopen(placemarks_filename.c_str(), "w");
                    if (fp != NULL) {
                        for (int i = 0;
							 i < (int)places.size(); i++) {
                            fprintf(fp, "%s, %f, %f, %f, %f\n",
									places[i].c_str(),
									poses[i*4], poses[i*4+1],
									poses[i*4+2], poses[i*4+3]);
                        }
                        fclose(fp);
                        saved = true;
                    }
                }

                if ((saved) && (learn_button_sound !="")) {
                    char saystr[255];
                    sprintf((char*)saystr,
							learn_button_sound.c_str(),
							place_names[current_place_name_index].c_str());
                    sc.say(saystr);
                }
            }
        }
    }
}


/*!
 * \brief Select the next place name
 */
void select_next_place_name()
{
    if (is_enabled()) {
        sound_play::SoundClient sc;
        if (place_names.size() > 0) {
            // increment the place name
            current_place_name_index++;
            if (current_place_name_index >=
				(int)place_names.size()) {
                current_place_name_index = 0;
            }
            // say the place name
            sc.say(place_names[current_place_name_index]);
        }
    }
}

/*!
 * \brief Set the current pose
 */
void set_current_pose()
{
    if (is_enabled()) {
        sound_play::SoundClient sc;
        if (place_names.size() > 0) {
            std::vector<std::string> places;
            std::vector<float> poses;
            int index = read_placemarks(places, poses);
            if (index > -1) {
                double theta=0;

                // create an initial pose
                geometry_msgs::PoseWithCovarianceStamped p =
					current_pose;                
                p.header.frame_id = "/map";
                p.header.stamp = ros::Time::now();
                p.pose.pose.position.x = poses[index*4];
                p.pose.pose.position.y = poses[index*4+1];
                p.pose.pose.orientation.z = poses[index*4+2];
                p.pose.pose.orientation.w = poses[index*4+3];
                for (int i = 0;
					 i < (int)p.pose.covariance.size(); i++) {
                    p.pose.covariance[i]=0;
                }
                p.pose.covariance[6*0+0] =
					initial_pose_radius * initial_pose_radius;
                p.pose.covariance[6*1+1] =
					initial_pose_radius * initial_pose_radius;
                p.pose.covariance[6*3+3] =
					initial_pose_orientation_variance *
					initial_pose_orientation_variance;

                // create a pose array
                /*
				  geometry_msgs::PoseArray poses;
				  poses.header.frame_id = "/base_link";
				  poses.header.stamp = ros::Time::now();
				  for (int p = 0; p < 2000; p++) {
				  geometry_msgs::Pose sample;
				  double dist = (rand()/(double)RAND_MAX)*initial_pose_radius;
				  double angle = M_PI*2*(rand()/(double)RAND_MAX);
				  sample.position.x = dist*sin(angle);
				  sample.position.y = dist*cos(angle);
				  sample.position.z=0;
				  angle = (initial_pose_orientation_variance*2*(rand()/(double)RAND_MAX))-initial_pose_orientation_variance;
				  sample.orientation = tf::createQuaternionMsgFromYaw(angle);
				  }

				  if (pose_array_topic !="") {
				  pose_array_pub.publish(poses);
				  }
                */

                if (initial_pose_topic !="") {
                    initial_pose_pub.publish(p);
                }

                char saystr[255];
                sprintf((char*)saystr,
						set_pose_button_sound.c_str(),
						place_names[current_place_name_index].c_str());
                sc.say(saystr);
            }
            else {
                ROS_WARN("No placemark found");
            }
        }
    }
}


void go()
{
    sound_play::SoundClient sc;
    if ((current_place_name_index > -1) && (!moving_to_goal)) {
        std::vector<std::string> places;
        std::vector<float> poses;

        int index = read_placemarks(places, poses);

        if (index > -1) {

            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.header.frame_id = "/map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = poses[index*4];
            goal.target_pose.pose.position.y = poses[index*4+1];
            goal.target_pose.pose.orientation.z = poses[index*4+2];
            goal.target_pose.pose.orientation.w = poses[index*4+3];

            ROS_INFO("Sending goal");

            // tell the action client that we want to spin
			// a thread by default
            if (ac==NULL) {
				ac = new MoveBaseClient("move_base", true);
			}

            //wait for the action server to come up
            while(!ac->waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base " \
						 "action server to come up");
            }
            ac->sendGoal(goal);

            moving_to_goal = true;

            char saystr[255];
            sprintf((char*)saystr,go_button_sound.c_str(),
					place_names[current_place_name_index].c_str());
            sc.say(saystr);
        }
    }
}

/*!
 * \brief callback when an input is activated
 * \param ptr sensor parameters
 */
void interfaceKitCallback(const phidgets::interface_kit_params::ConstPtr& ptr)
{
    if (initialised) {
        if (is_enabled()) {
            phidgets::interface_kit_params ifk = *ptr;
            // if this is a digital input with a non-zero state
            if ((ifk.value_type == 1) && (ifk.value != 0)) {            
                if ((ifk.index ==
					 go_button_input_index) && (started)) {                
                    ROS_INFO("Go button pressed");
                    go();                    
                }
                if ((ifk.index ==
					 learn_button_input_index) && (started)) {
                    ROS_INFO("Learn button pressed");
                    if ((ros::Time::now()-previous_learn_button_press).toSec()<3) {
                        learn_pose();
                    }
                    previous_learn_button_press =
						ros::Time::now();
                }
                if ((ifk.index ==
					 place_button_input_index) && (started)) {                
                    ROS_INFO("Selecting place name");
                    select_next_place_name();
                }
                if ((ifk.index ==
					 set_pose_button_input_index) && (started)) {                
                    ROS_INFO("Set current pose");
                    set_current_pose();
                }
                if ((ifk.index ==
					 start_button_input_index) && (!started)) {
                    if (previous_button==set_pose_button_input_index) {
                        // Pressing start moves to the next location
                        current_place_name_index++;
                        if (current_place_name_index >=
							(int)place_names.size()) {
                            current_place_name_index = 0;
                        }
                    }
                    started = true;
                    ROS_INFO("Start button pressed");
                }
                if ((ifk.index ==
					 stop_button_input_index) && (started)) {
                    started = false;
                    ROS_INFO("Stop button pressed");
                }
                previous_button = ifk.index;
            }
        }
    }
}

/*!
 * \brief callback when a new pose estimate is received
 * \param ptr pose
 */
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ptr)
{
    if (initialised) {
        geometry_msgs::PoseWithCovarianceStamped p = *ptr;
        current_pose = p;
        ROS_INFO("Pose received %.3f, %.3f, %.3f, %.3f",
				 p.pose.pose.position.x,
				 p.pose.pose.position.y,
				 p.pose.pose.orientation.z,
				 p.pose.pose.orientation.w);
        if (initial_pose_set) {
			save_current_pose(current_pose_filename);
		}
        pose_received = true;
    }
}


/*!
 * \brief callback when the motor parameters change
 * \param ptr motor parameters
 */
void motorCallback(const phidgets::motor_params::ConstPtr& ptr)
{
    phidgets::motor_params m = *ptr;
    switch(m.value_type) {
	case 1: { // inputs
		//ROS_INFO("Motor input %d State %.2f", m.index, m.value);
		break;
	}
	case 2: { // velocity
		//ROS_INFO("Motor %d Velocity %.2f", m.index, m.value);
		break;
	}
	case 3: { // current
		//ROS_INFO("Motor %d Current %.2f", m.index, m.value);
		break;
	}
	}    
}

/*!
 * \brief sets all velocities to zero
 */
void stop_motors()
{
    if (initialised) {
        cmd.linear.x = 0;
        cmd.linear.y = 0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = 0;
    }

    ROS_INFO("Motors stopped");
}

/*!
 * \brief joystick command has arrived
 */
void joystickCallback(const phidgets::joystick_params::ConstPtr& ptr)
{
    if (initialised) {
        if (is_enabled()) {
            if ((start_button_input_index == -1) ||
                (stop_button_input_index == -1) ||
                ((start_button_input_index != -1) &&
				 (stop_button_input_index != -1) &&
				 (started))) {
                // get the joystick values
                phidgets::joystick_params joy = *ptr;
                float rotate = joy.axes[0];
                float forward = joy.axes[1];
                bool button_press = false;
                if (joy.buttons[0] != 0) button_press = true;
                float deadband2 = deadband;

                // Make the deadband bigger if moving to a goal
                if (moving_to_goal) deadband2 *= 2;

                if ((fabs(rotate) > deadband2) || 
                    (fabs(forward) > deadband2)) {

                    if (moving_to_goal) {
                        joystick_control_resumed = true;
                        ac->cancelGoal();
                    }
                    else {
                        // joystick moved
                        // vector values should be in the range -1 <= x <= 1
                        cmd.linear.x =
							forward*max_linear_velocity;
                        cmd.angular.z =
							rotate*max_angular_velocity;
                        ROS_INFO("Joystick command forward %f/%f",
								 forward, cmd.linear.x);
                        ROS_INFO("Joystick command rotate %f/%f",
								 rotate, cmd.angular.z);
                    }
                }
                else {
                    // joystick centred
                    if (!moving_to_goal) stop_motors();
                }
            }
        }
    }
}

/*
 * \brief Loads the initial pose from file.  This allows the robot to know its initial location after being switched on.
 */
void load_initial_pose(std::string filename)
{
    FILE * fp = fopen(filename.c_str(),"r");
    if (fp != NULL) {
        geometry_msgs::PoseWithCovarianceStamped p = current_pose;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = "/map";
        int index=0;
        std::string str="";
        while (!feof(fp)) {
            char c = fgetc(fp);
            if ((c == ',') || (c == '\n')) {
                if (str != "") {
                    switch(index) {
					case 0: { p.pose.pose.position.x =
								atof(str.c_str()); break; };
					case 1: { p.pose.pose.position.y =
								atof(str.c_str()); break; };
					case 2: { p.pose.pose.orientation.z =
								atof(str.c_str()); break; };
					case 3: { p.pose.pose.orientation.w =
								atof(str.c_str()); break; };
                    }
                    index++;
                    str="";
                }
            }
            else {
                str += c;
            }
        }
        if (str != "") {
            p.pose.pose.orientation.w = atof(str.c_str());
        }

        p.pose.covariance[6*0+0] =
			initial_pose_radius * initial_pose_radius;
        p.pose.covariance[6*1+1] =
			initial_pose_radius * initial_pose_radius;
        p.pose.covariance[6*3+3] =
			initial_pose_orientation_variance *
			initial_pose_orientation_variance;

        fclose(fp);
        if (initial_pose_topic !="") {
            ROS_INFO("Publishing initial pose %s " \
					 "%.3f, %.3f,%.3f,%.3f to %s",
					 p.header.frame_id.c_str(),
					 p.pose.pose.position.x,
					 p.pose.pose.position.y,
					 p.pose.pose.orientation.z,
					 p.pose.pose.orientation.w,
					 initial_pose_topic.c_str());
            initial_pose_pub.publish(p);
        }
        current_pose = p;
    }
    else {
        ROS_WARN("No known current location");
    }
}

/*!
 * \brief loads place names from a comma separated string.
 *        eg. Kitchen table,Lounge,Hallway
 * \param placenames string containing place names
 */
void load_place_names(std::string placenames)
{
    place_names.clear();
    if (placenames != "") {
        const char * place_names_str = placenames.c_str();
        std::string str="";
        for (int i = 0; i < (int)placenames.length(); i++) {
            char c = place_names_str[i];
            if (c != ',') {
                str += c;
            }
            else {
                if (str != "") {
                    place_names.push_back(str);
                    str = "";
                }
            }
        }
        if (str != "") {
            place_names.push_back(str);
        }
    }
}

int main(int argc, char** argv)
{
    ROS_INFO("Joystick control of Phidgets Motor Control HC");
    ros::init(argc, argv, "joystick_motor_control");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    
    sound_play::SoundClient sc;
    sleep(4); // pause to wait for the sound client to connect

    bool enable = false;
    nh.getParam("enable", enable);

    std::string name = "cmd_vel";
    nh.getParam("name", name);
    std::string topic_path = "phidgets/";
    nh.getParam("topic_path", topic_path);

    std::string joystick_name = "joystick";
    nh.getParam("joystickname", joystick_name);
    std::string motor_control_name = "motorcontrol";
    nh.getParam("motorcontrolname", motor_control_name);
    std::string interface_kit_name = "interface_kit";
    nh.getParam("interfacekitname", interface_kit_name);

    double vel=0;
    nh.getParam("max_linear_velocity", vel);
    if (vel>0) max_linear_velocity = vel;
    nh.getParam("max_angular_velocity", vel);
    if (vel>0) max_angular_velocity = vel;

    // sounds
    std::string sound_str = "";
    nh.getParam("startbuttonsound", sound_str);
    if (sound_str != "") start_button_sound = sound_str;
    sound_str = "";
    nh.getParam("stopbuttonsound", sound_str);
    if (sound_str != "") stop_button_sound = sound_str;
    sound_str = "";
    nh.getParam("enablesound", sound_str);
    if (sound_str != "") enabled_sound = sound_str;
    sound_str = "";
    nh.getParam("learnbuttonsound", sound_str);
    if (sound_str != "") learn_button_sound = sound_str;
    sound_str = "";
    nh.getParam("gobuttonsound", sound_str);
    if (sound_str != "") go_button_sound = sound_str;

    // start and stop button digital inputs on the interface kit
    nh.getParam("startbutton", start_button_input_index);
    nh.getParam("startbuttoncommand", start_button_command);
    if ((start_button_input_index < 0) ||
		(start_button_input_index > 16)) {
		start_button_input_index = -1;
	}
    nh.getParam("stopbutton", stop_button_input_index);
    nh.getParam("stopbuttoncommand", stop_button_command);
    if ((stop_button_input_index < 0) ||
		(stop_button_input_index > 16)) {
		stop_button_input_index = -1;
	}
    if (start_button_input_index == stop_button_input_index) {
        start_button_input_index = -1;
        stop_button_input_index = -1;
    }

    // Button for learning new locations/poses
    std::string placenames = "";
    nh.getParam("pose_topic", pose_topic);
    nh.getParam("pose_array_topic", pose_array_topic);
    nh.getParam("initial_pose_topic", initial_pose_topic);
    nh.getParam("current_pose_filename", current_pose_filename);
    nh.getParam("placemarks", placemarks_filename);
    nh.getParam("placenames", placenames);
    nh.getParam("setposebutton", set_pose_button_input_index);
    nh.getParam("placebutton", place_button_input_index);
    nh.getParam("learnbutton", learn_button_input_index);
    nh.getParam("gobutton", go_button_input_index);
    nh.getParam("learnbuttoncommand", learn_button_command);
    if ((learn_button_input_index < 0) ||
		(learn_button_input_index > 16)) {
		learn_button_input_index = -1;
	}

    // speed and acceleration of the motors
    int v;
    nh.getParam("speed", v);
    if ((v > 1) && (v < 100)) speed = (float)v;
    nh.getParam("acceleration", v);
    if ((v > 1) && (v < 100)) acceleration = (float)v;

    // joystick deadband
    nh.getParam("deadband", v);
    if ((v > 0.001) && (v < 1.0)) deadband = (float)v;

    int frequency = 20;
    nh.getParam("frequency", frequency);

    // pose uncertainty
    double val=0;
    nh.getParam("initial_pose_radius", val);
    if (val > 0.001) initial_pose_radius = val;
    nh.getParam("initial_pose_orientation_variance", val);
    if (val > 0.001) {
		initial_pose_orientation_variance = val*M_PI/180.0;
	}

    // subscribe to poses
    ros::Subscriber poses_sub;
    if (pose_topic != "") {
        poses_sub = n.subscribe(pose_topic, 1, poseCallback);
    }

    // subscribe to the motor control node
    ros::Subscriber motor_control_sub =
		n.subscribe(topic_path + motor_control_name,
					1, motorCallback);

    // subscribe to the joystick node
    ros::Subscriber joystick_sub =
		n.subscribe(topic_path + joystick_name,
					1, joystickCallback);

    // subscribe to the interface kit node
    ros::Subscriber interface_kit_sub =
		n.subscribe(topic_path + interface_kit_name,
					1, interfaceKitCallback);

    // publish motor commands in geometry_msgs::Twist format
    motor_pub = n.advertise<geometry_msgs::Twist>(name, 10);

    // initial pose publisher
    if (initial_pose_topic != "") {
        initial_pose_pub =
			n.advertise<geometry_msgs::PoseWithCovarianceStamped>(initial_pose_topic, 10);
    }

    // pose array publisher
    if (pose_array_topic != "") {
        pose_array_pub =
			n.advertise<geometry_msgs::PoseArray>(pose_array_topic, 10);
    }

    // halt the motors
    stop_motors();

    // make the joystick active
    n.setParam("joystick/enable", true);

    n.setParam("joystick/enable_motor_control", enable);

    if (!enable) {
        ROS_INFO("Waiting for enable parameter to be set");
    }
    else {
        if (enabled_sound !="") sc.say(enabled_sound);
    }

    // load place names
    load_place_names(placenames);

    // load the initial pose
    load_initial_pose(current_pose_filename);

    ros::Rate loop_rate(frequency);
    initialised = true;
    bool prev_enable = enable;
    bool prev_started = started;
    char saystr[100];
    while (ros::ok()) {
        // has joystick control been enabled ?
        bool en = is_enabled();

        if ((pose_received) && (!initial_pose_set)) {
            load_initial_pose(current_pose_filename);
            initial_pose_set = true;
        }

        if ((en) && (!moving_to_goal)) motor_pub.publish(cmd);
        ros::spinOnce();
        loop_rate.sleep();

        // speak message to indicate that joystick control has
        // started or stopped
        if ((started) && (!prev_started)) {
            // load the initial pose
            if (start_button_sound !="") {
				sc.say(start_button_sound);
			}
            if (start_button_command != "") {
                ROS_INFO("Running command:\n%s",
						 start_button_command.c_str());
                system(start_button_command.c_str());
            }
        }
        if ((!started) && (prev_started)) {
            if (stop_button_sound !="") {
				sc.say(stop_button_sound);
			}
            if (stop_button_command != "") {
                ROS_INFO("Running command:\n%s",
						 stop_button_command.c_str());
                system(stop_button_command.c_str());
            }
        }
        if ((en) && (!prev_enable)) {
            if (enabled_sound !="") sc.say(enabled_sound);
        }

        if (moving_to_goal) {
            if (ac->getState() ==
				actionlib::SimpleClientGoalState::SUCCEEDED) {
                sprintf((char*)saystr,arrived_goal_sound.c_str(),place_names[current_place_name_index].c_str());
                ROS_INFO((char*)saystr);
                sc.say(saystr);
                // Proceed to the next location
                current_place_name_index++;
                if (current_place_name_index>=place_names.size()) current_place_name_index=0;
                moving_to_goal = false;
            }
            if ((ac->getState() ==
				 actionlib::SimpleClientGoalState::LOST) ||
                (ac->getState() ==
				 actionlib::SimpleClientGoalState::REJECTED) ||
                (ac->getState() ==
				 actionlib::SimpleClientGoalState::ABORTED)) {
                if (!joystick_control_resumed) {
                    sprintf((char*)saystr,
							failed_goal_sound.c_str(),
							place_names[current_place_name_index].c_str());
                    ROS_INFO((char*)saystr);
                }
                else {
                    sprintf((char*)saystr,
							joystick_resumed_sound.c_str());
                    joystick_control_resumed = false;
                    ROS_INFO("Joystick control resumed");
                }
                sc.say(saystr);
                moving_to_goal = false;
            }
        }

        // if joystick control has started and the
		// enable flag goes low
        // then ensure that the motors are stopped
        if ((!en) && (prev_enable) && (started)) {
            started = false;
            stop_motors();
        }
        if ((en) && (!started) && (prev_started)) {
            stop_motors();
        }
        prev_enable = en;
        prev_started = started;
    }

    if (ac!=NULL) delete ac;

    return 0;
}

