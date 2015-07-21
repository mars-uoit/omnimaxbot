/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Lidar using an IR range sensor and advanced servo controller
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
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include "phidgets/servo_reference.h"
#include "phidgets/servo_params.h"
#include "phidgets/interface_kit_params.h"
#include "phidgets/lidar_control.h"

using namespace std;

#define PHIDGETS_ADVANCED_SERVO_MOTORS 8
#define LIDAR_MAX_SENSORS              8

#define SENSOR_SHARP_GP2Y0A700K0F      0
#define SENSOR_SHARP_GP2Y0A2JYK        1
#define SENSOR_SHARP_GP2D120           2
#define SENSOR_SHARP_GP2D12            3
#define SENSOR_SHARP_GP2Y0A02YK        4

vector<ros::Time> sensor_value_time[LIDAR_MAX_SENSORS];
vector<float> sensor_value0[LIDAR_MAX_SENSORS];
vector<float> sensor_value1[LIDAR_MAX_SENSORS];
vector<float> sensor_coupling;

vector<ros::Time> servo_time;
vector<float> servo_angle;

ros::ServiceClient client_servo_reference;
sensor_msgs::LaserScan scan[10];

// index numbers of the servos
int pan_servo_index = 0;
int tilt_servo_index = 1;

// index of the IR sensor on the interface kit
int range_sensor_index = 0;

// the number of range sensors on the LIDAR
int no_of_sensors = 1;

// the type of range sensor used
vector<int> sensor_type;

// servo positions for the min and max scan angles
float pan_position_left = 100;
float pan_position_left_angle_degrees = -45;
float pan_position_right = 160;
float pan_position_right_angle_degrees = 45;
float tilt_position_up = 100;
float tilt_position_up_angle_degrees = 0;
float tilt_position_down = 140;
float tilt_position_down_angle_degrees = -50;

float current_tilt_position = 0;
int tilt_step = 0;
int no_of_tilt_steps = 0;
int tilt_step_direction = 1;

// speed and acceleration of the servo
float speed=30;
float acceleration=100;

// cycle time for the scan
float cycle_time_sec = 5;

// suppress the first move
bool suppress = true;

bool paused = true;
bool ignore_sensors = false;

// publisher for LaserScan messages
ros::Publisher lidar_pub;

// position of the LIDAR relative to the robot base in metres
double lidar_x_m=0, lidar_y_m=0, lidar_z_m=0;
// rotation of the lidar relative to the robot base in radians
double lidar_rotation_radians = 0;
// tilt of the lidar relative to the robot base in radians
double lidar_tilt_radians = 0;
// distance between each sensor and the centre of
// rotation of the LIDAR base
double sensor_dist_from_centre_of_rotation_m = 0.03;
// time when LIDAR transforms were last published
ros::Time transforms_last_published;

int publish_scan_index = 0;

/*!
 * \brief returns the minimum range for the sensor (peak voltage) in metres
 * \param sensor_value array containing the sensor profile graph
 * \param points number of points in the sensor profile graph
 * \param width_pixels width of the sensor profile graph in pixels
 * \param height_pixels height_pixels
 * \param width_mm width of the sensor profile graph in millimetres
 * \param height_volts height of the sensor profile graph in volts
 * \param max_value value from the interface kit at 5 volts input
 * \return minimum range in metres
 */
float get_minimum_range(const int* sensor_profile,
						int points,
						int width_pixels,
						int height_pixels,
						float width_mm,
						float height_volts,
						float max_value)
{
    float minimum_range_metres = 0;

    float prev_volts = 0, peak_volts = 0;
    for (int i = 0; i < points; i++) {
        float volts =
			sensor_profile[i*2+1] * height_volts /
			height_pixels;
        if (volts > peak_volts) {
            minimum_range_metres =
				sensor_profile[i*2] * width_mm /
				(width_pixels * 1000.0f);
            peak_volts = volts;
        }
        if ((i > 0) && (prev_volts > volts)) {
            break;
        }
        prev_volts = volts;
    }
    //ROS_INFO("minimum_range_metres %f", minimum_range_metres);
    return(minimum_range_metres);
}

/*!
 * \brief convert a sensor value into a range
 * \param value sensor value to be converted
 * \param sensor_value array containing the sensor profile graph
 * \param points number of points in the sensor profile graph
 * \param width_pixels width of the sensor profile graph in pixels
 * \param height_pixels height_pixels
 * \param width_mm width of the sensor profile graph in millimetres
 * \param height_volts height of the sensor profile graph in volts
 * \param max_value value from the interface kit at 5 volts input
 * \return range in metres (two values)
 */
void sensor_value_to_range(int value,
						   const int* sensor_profile,
						   float interface_kit_sensor_max_voltage,
						   int points,
						   int width_pixels,
						   int height_pixels,
						   float width_mm,
						   float height_volts,
						   float max_value,
						   float &range_metres0,
						   float &range_metres1)
{
    // get the minimum range at which the voltage peaks
    float minimum_range_metres = 
        get_minimum_range(
						  sensor_profile,
						  points,
						  width_pixels,
						  height_pixels,
						  width_mm,
						  height_volts,
						  max_value);

    // two possible range solutions
    range_metres0 = 0;
    range_metres1 = 0;

    // convert sensor value to a voltage
    float volts =
		value * interface_kit_sensor_max_voltage /
		max_value;

    float prev_v = 0, prev_dist = 0;
    for (int i = 0; i < points; i++) {
        float v =
			sensor_profile[i*2+1] * height_volts /
			height_pixels;
        float dist =
			sensor_profile[i*2] * width_mm /
			width_pixels;
        if ((((v > prev_v) && (volts >= prev_v) &&
			  (volts <= v)) || 
			 ((v < prev_v) && (volts >= v) &&
			  (volts <= prev_v))) &&
            (i > 0)) {
            float fraction = (volts - prev_v) / (v - prev_v);
            float d = (prev_dist + ((dist - prev_dist) *
									fraction)) / 1000.0f;
            if (dist < minimum_range_metres) {
                // less than minimum range
                range_metres0 = d;
            }
            else {
                // greater than minimum range
                range_metres1 = d;
            }
        }
        prev_v = v;
        prev_dist = dist;
    }
    //ROS_INFO("Volts %f  Range %f %f m",
	//         volts, range_metres0, range_metres1);
}

/*!
 * \brief convert a sensor value into a range for the Sharp GP2Y0A02YK
 * \return range in metres
 */
void sharp_GP2Y0A02YK_sensor_value_to_range(int value,
											float &range_metres0,
											float &range_metres1)
{
    // two possible range solutions
    range_metres0 = 0;
    range_metres1 = 0;

    const int sensor_profile[] = {
        15,145,  25,258,  35,304,  47,280,
        62,236,  75,205,  91,168,  113,135,
        157,99,  201,80,  245,65,  289,56,
        333,46
    };

    float interface_kit_sensor_max_voltage = 5.0f;
	// number of points in the sensor profile graph
    int points = 4*3+1;
	// width of the sensor profile graph in pixels
    int width_pixels = 333;
	// height of the sensor profile graph in pixels
    int height_pixels = 332;
	// width of the sensor profile graph in millimetres
    float width_mm = 1500;
	// height of the sensor profile graph in volts
    float height_volts = 3.0f;
	// value from the interface kit at 5 volts input
    float max_value = 1000;

    sensor_value_to_range(
						  value,
						  sensor_profile,
						  interface_kit_sensor_max_voltage,
						  points,
						  width_pixels,
						  height_pixels,
						  width_mm,
						  height_volts,
						  max_value,
						  range_metres0,
						  range_metres1);
}

/*!
 * \brief convert a sensor value into a range for the Sharp GP2D12 / GP2D15
 * \return range in metres
 */
void sharp_GP2D12_sensor_value_to_range(int value,
										float &range_metres0,
										float &range_metres1)
{
    // two possible range solutions
    range_metres0 = 0;
    range_metres1 = 0;

    const int sensor_profile[] = {
        28,267,  30,282,  34,287,  41,275,
        46,250,  56,209,  79,159,  103,125,
        128,105,  158,88,  193,73,  229,64,
        268,56,  296,51,  332,45
    };

    float interface_kit_sensor_max_voltage = 5.0f;
	// number of points in the sensor profile graph
    int points = 4*3+3;
	// width of the sensor profile graph in pixels
    int width_pixels = 332;
	// height of the sensor profile graph in pixels
    int height_pixels = 331;
	// width of the sensor profile graph in millimetres
    float width_mm = 800;
	// height of the sensor profile graph in volts
    float height_volts = 3.0f;
	// value from the interface kit at 5 volts input
    float max_value = 1000;

    sensor_value_to_range(
						  value,
						  sensor_profile,
						  interface_kit_sensor_max_voltage,
						  points,
						  width_pixels,
						  height_pixels,
						  width_mm,
						  height_volts,
						  max_value,
						  range_metres0,
						  range_metres1);
}

/*!
 * \brief convert a sensor value into a range for the Sharp GP2D120
 * \return range in metres
 */
void sharp_GP2D120_sensor_value_to_range(
										 int value,
										 float &range_metres0,
										 float &range_metres1)
{
    // two possible range solutions
    range_metres0 = 0;
    range_metres1 = 0;

    const int sensor_profile[] = {
        3,112,  6,144,  7,157,  13,178,
        16,208,  20,258,  25,253,  28,221,
        34,187,  47,146,  66,108,  92,81,
        118,64,  155,49,  188,41,  225,34,
        251,28,  285,25
    };

    float interface_kit_sensor_max_voltage = 5.0f;
	// number of points in the sensor profile graph
    int points = 4*4+2;
	// width of the sensor profile graph in pixels
    int width_pixels = 285;
	// height of the sensor profile graph in pixels
    int height_pixels = 284;
	// width of the sensor profile graph in millimetres
    float width_mm = 400;
	// height of the sensor profile graph in volts
    float height_volts = 3.4f;
	// value from the interface kit at 5 volts input
    float max_value = 1000;

    sensor_value_to_range(value,
						  sensor_profile,
						  interface_kit_sensor_max_voltage,
						  points,
						  width_pixels,
						  height_pixels,
						  width_mm,
						  height_volts,
						  max_value,
						  range_metres0,
						  range_metres1);
}

/*!
 * \brief convert a sensor value into a range for the Sharp GP2Y0A2JYK
 * \return range in metres
 */
void sharp_GP2Y0A2JYK_sensor_value_to_range(int value,
											float &range_metres0,
											float &range_metres1)
{
    // two possible range solutions
    range_metres0 = 0;
    range_metres1 = 0;

    const int sensor_profile[] = {
        8,94,  13,167,  19,258,  21,294,
        26,297,  31,277,  44,211,  63,154,
        86,122,  109,96,  131,83,  165,70,
        202,58,  247,48,  291,40,  333,37
    };

    float interface_kit_sensor_max_voltage = 5.0f;
	// number of points in the sensor profile graph
    int points = 4*4;
	// width of the sensor profile graph in pixels
    int width_pixels = 333;
	// height of the sensor profile graph in pixels
    int height_pixels = 330;
	// width of the sensor profile graph in millimetres
    float width_mm = 800;
	// height of the sensor profile graph in volts
    float height_volts = 3.5f;
	// value from the interface kit at 5 volts input
    float max_value = 1000;

    sensor_value_to_range(
						  value,
						  sensor_profile,
						  interface_kit_sensor_max_voltage,
						  points,
						  width_pixels,
						  height_pixels,
						  width_mm,
						  height_volts,
						  max_value,
						  range_metres0,
						  range_metres1);
}

/*!
 * \brief convert a sensor value into a range for the Sharp GP2Y0A700K0F
 * \return range in metres
 */
void sharp_GP2Y0A700K0F_sensor_value_to_range(int value,
											  float &range_metres0,
											  float &range_metres1)
{
    // two possible range solutions
    range_metres0 = 0;
    range_metres1 = 0;

    const int sensor_profile[] = {
        14,114,  16,126,  22,142,  27,165,
        31,184,  36,212,  42,239,  45,253,
        53,263,  59,260,  69,246,  80,229,
        91,204,  104,180,  119,161,  141,139,
        165,118,  193,101,  225,88,  263,75,
        306,67,  339,62,  371,55,  420,49
    };
    float interface_kit_sensor_max_voltage = 5.0f;
	// number of points in the sensor profile graph
    int points = 4*6;
	// width of the sensor profile graph in pixels
    int width_pixels = 456;
	// height of the sensor profile graph in pixels
    int height_pixels = 310;
	// width of the sensor profile graph in millimetres
    float width_mm = 6000;
	// height of the sensor profile graph in volts
    float height_volts = 3.5f;
	// value from the interface kit at 5 volts input
    float max_value = 1000;

    sensor_value_to_range(value,
						  sensor_profile,
						  interface_kit_sensor_max_voltage,
						  points,
						  width_pixels,
						  height_pixels,
						  width_mm,
						  height_volts,
						  max_value,
						  range_metres0,
						  range_metres1);
}

/*!
 * \brief extracts scans from the stream of servo and asensor data
 * \param sensor_index index number of the sensor on the LIDAR
 */
void extract_scan(
				  int sensor_index,
				  int length,
				  sensor_msgs::LaserScan &laser_scan)
{
    // tollerance when matching sensor ranges
    const float tollerance_metres = 0.3f;

    laser_scan.angle_min = servo_angle[0];
    laser_scan.angle_max = servo_angle[length-3];
    laser_scan.scan_time =
		(servo_time[length-3] - servo_time[0]).toSec();
    int increments = length-3;
    laser_scan.time_increment =
		laser_scan.scan_time / (float)increments;
    laser_scan.angle_increment =
		(laser_scan.angle_max - laser_scan.angle_min) /
		(float)increments;
    laser_scan.header.stamp = servo_time[0];

    // create a frame identifier for this sensor
    char frame_id_str[50];
    sprintf(frame_id_str, "/base_lidar/lidar_sensor_%d",
			sensor_index);
    std::string frame_id_str2 = "";
    frame_id_str2 += frame_id_str;
    laser_scan.header.frame_id = frame_id_str2;

    // update the range values
    laser_scan.ranges.clear();
    int j = 0;
    int k = 0;
    int max = sensor_value_time[sensor_index].size()-1;
    if (max > 1) {
        for (int i = 0; i < increments; i++) {
            while ((j < max) &&
				   (sensor_value_time[sensor_index][j] <
					servo_time[i])) {
                j++;
            }
            float range0 = sensor_value0[sensor_index][j];
            float range1 = sensor_value1[sensor_index][j];

            int coupled_sensor_index =
				sensor_coupling[sensor_index];
            if ((coupled_sensor_index > -1) && 
                (coupled_sensor_index < no_of_sensors)) {

				ROS_INFO("1");
                int max2 =
					sensor_value_time[coupled_sensor_index].size()-1;
				ROS_INFO("2");
                while ((k < max2) &&
					   (sensor_value_time[coupled_sensor_index][k] <
						servo_time[i])) {
                    k++;
                }
				ROS_INFO("3");
                float diff =
					range0 -
					sensor_value1[coupled_sensor_index][k];
				ROS_INFO("4");
                if ((diff > -tollerance_metres) &&
					(diff < tollerance_metres)) {
                    range1 =
						(range0 +
						 sensor_value1[coupled_sensor_index][k])*0.5f;
                    ROS_INFO("Composite range %f", range1);
                }
                else {
                    ROS_INFO("Ranges %f  %f",
							 range1,
							 sensor_value1[coupled_sensor_index][k]);
                }
				ROS_INFO("5");
            }

            laser_scan.ranges.push_back(range1);
        }
            
        // clear sensor values
        vector<float> sens = sensor_value0[sensor_index];
        sens.erase(sens.begin(), sens.begin()+j);
        sens = sensor_value1[sensor_index];
        sens.erase(sens.begin(), sens.begin()+k);

        vector<ros::Time> sens2 =
			sensor_value_time[sensor_index];
        sens2.erase(sens2.begin(), sens2.begin()+j);

        // if this is the last sensor index then clear up
        if (sensor_index == no_of_sensors-1) {
            servo_angle.erase(servo_angle.begin(),
							  servo_angle.begin()+increments);
            servo_time.erase(servo_time.begin(),
							 servo_time.begin()+increments);

            publish_scan_index = no_of_sensors;
            ROS_INFO("Direction changed");
        }
    }
}

/*!
 * \brief callback when the sensor or output values change
 * \param ptr sensor parameters
 */
void interfaceKitCallback(const phidgets::interface_kit_params::ConstPtr& ptr)
{
    phidgets::interface_kit_params ifk = *ptr;
    if ((!suppress) &&
		((ifk.value_type == 3) && 
		 (ifk.index >= range_sensor_index) && 
		 (ifk.index < range_sensor_index + no_of_sensors))) {
        
        float range_metres0 = 0;
        float range_metres1 = 0;

        // convert the sensor value into a range
        int idx = ifk.index - range_sensor_index;
        if ((idx > -1) && (idx < no_of_sensors)) {
            switch(sensor_type[idx]) {
			case SENSOR_SHARP_GP2Y0A700K0F: {
				sharp_GP2Y0A700K0F_sensor_value_to_range(
														 ifk.value, 
														 range_metres0,
														 range_metres1);
				break;
			}
			case SENSOR_SHARP_GP2Y0A2JYK: {
				sharp_GP2Y0A2JYK_sensor_value_to_range(
													   ifk.value, 
													   range_metres0,
													   range_metres1);
				break;
			}
			case SENSOR_SHARP_GP2D120: {
				sharp_GP2D120_sensor_value_to_range(
													ifk.value, 
													range_metres0,
													range_metres1);
				break;
			}
			case SENSOR_SHARP_GP2D12: {
				sharp_GP2D12_sensor_value_to_range(
												   ifk.value, 
												   range_metres0,
												   range_metres1);
				break;
			}
			case SENSOR_SHARP_GP2Y0A02YK: {
				sharp_GP2Y0A02YK_sensor_value_to_range(
													   ifk.value, 
													   range_metres0,
													   range_metres1);
				break;
			}
			}
 
            if ((!ignore_sensors) && (range_metres1 > 0)) {
                sensor_value_time[ifk.index - range_sensor_index].push_back(ros::Time::now());
                sensor_value0[ifk.index - range_sensor_index].push_back(range_metres0);
                sensor_value1[ifk.index - range_sensor_index].push_back(range_metres1);
            }
        }
    }
}

/*!
 * \brief callback when a servo has changed position
 * \param servo parameters
 */
void servoCallback(const phidgets::servo_params::ConstPtr& ptr)
{
    phidgets::servo_params s = *ptr;
    if ((!suppress) && (s.index == pan_servo_index)) {
        servo_time.push_back(ros::Time::now());

        float angular_range =
			pan_position_right_angle_degrees -
			pan_position_left_angle_degrees;
        float position_range =
			pan_position_right - pan_position_left;
        float servo_ang =
			pan_position_left_angle_degrees +
			((s.position - pan_position_left) *
			 angular_range / position_range);
        servo_ang = servo_ang * 3.1415927f / 180.0f;
        servo_angle.push_back(servo_ang);
        //ROS_INFO("Position %f  Angle %f",
		//         s.position, servo_ang);
        ignore_sensors = true;

        if ((int)servo_time.size() > 6) {
            int length = (int)servo_time.size();
            float diff0 = servo_angle[length-1] -
				servo_angle[length - 3];
            float diff1 = servo_angle[length-3] -
				servo_angle[length - 5];
            if (((diff0 < 0) && (diff1 > 0)) ||
                ((diff0 > 0) && (diff1 < 0))) {

                for (int sensor_index = 0;
					 sensor_index < no_of_sensors;
					 sensor_index++) {
                    extract_scan(sensor_index, length,
								 scan[sensor_index]);
                }
            }
        }
        ignore_sensors = false;
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

/*!
 * \brief publish transforms between the robot base and the LIDAR sensor/s
 * \param transform_broadcaster publisher for transforms
 */
void publish_transforms(tf::TransformBroadcaster &transform_broadcaster)
{
    double time_since_last_published_sec = (ros::Time::now() - transforms_last_published).toSec();
    if (time_since_last_published_sec > 1) {

        ros::Time current_time = ros::Time::now();

        // broadcast transform between the LIDAR
		// and the robot base
        transform_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(
																			   tf::Quaternion(lidar_tilt_radians, 0,
																							  lidar_rotation_radians, 1), 
																			   tf::Vector3(lidar_x_m, lidar_y_m, lidar_z_m)),
																 current_time,"/base_link", "/base_lidar"));

        // broadcast the transform between each sensor
		// and the lidar base
        char frame_id_str[50];
        for (int sensor_index = 0;
			 sensor_index < no_of_sensors; sensor_index++) {
            double sensor_orientation_radians =
				sensor_index * 2 * 3.1415927 / no_of_sensors;
            double sensor_x_m =
				sensor_dist_from_centre_of_rotation_m *
				(float)sin(sensor_orientation_radians);
            double sensor_y_m =
				sensor_dist_from_centre_of_rotation_m *
				(float)cos(sensor_orientation_radians);

            // frame identifier for this sensor
            sprintf(frame_id_str,
					"/base_lidar/lidar_sensor_%d",
					sensor_index);
            std::string frame_id_str2 = "";
            frame_id_str2 += frame_id_str;

            transform_broadcaster.sendTransform(
												tf::StampedTransform(
																	 tf::Transform(
																				   tf::Quaternion(0, 0,
																								  sensor_orientation_radians, 1), 
																				   tf::Vector3(sensor_x_m, sensor_y_m, 0)),
																	 current_time,"/base_lidar", frame_id_str2));
        }

        transforms_last_published = ros::Time::now();
    }
}

/*!
 * \brief service changes the lidar parameters
 * \param req requested parameters
 * \param res returned parameters
 */ 
bool change_lidar_params(phidgets::lidar_control::Request &req,
						 phidgets::lidar_control::Response &res)
{
    if (!req.pause) {
        ROS_INFO("LIDAR running");    
    }
    else {
        ROS_INFO("LIDAR paused");
    }
    paused = req.pause;
    pan_servo_index = req.pan_servo_index;
    tilt_servo_index = req.tilt_servo_index;
    range_sensor_index = req.sensor_index;
    no_of_sensors = req.no_of_sensors;
    sensor_type.clear();
    sensor_coupling.clear();
    for (int s = 0; s < (int)req.sensor_type.size(); s++) {
        sensor_type.push_back(req.sensor_type[s]);
        if (s < (int)req.sensor_coupling.size()) {
            sensor_coupling.push_back(req.sensor_coupling[s]);
        }
        else {
            sensor_coupling.push_back(-1);
        }
    }
    if ((int)req.speed != 0) speed = req.speed;
    if ((int)req.acceleration != 0) {
		acceleration = req.acceleration;
	}

    pan_position_left = req.pan_position_left;
    pan_position_right = req.pan_position_right;
    pan_position_left_angle_degrees =
		req.pan_position_left_angle_degrees;
    pan_position_right_angle_degrees =
		req.pan_position_right_angle_degrees;

    tilt_position_up = req.tilt_position_up;
    tilt_position_down = req.tilt_position_down;
    tilt_position_up_angle_degrees =
		req.tilt_position_up_angle_degrees;
    tilt_position_down_angle_degrees =
		req.tilt_position_down_angle_degrees;
    no_of_tilt_steps = req.tilt_steps;

    cycle_time_sec = req.cycle_time_sec;
    if (cycle_time_sec < 2) cycle_time_sec  = 2;
    tilt_step = 0;
    tilt_step_direction = 1;
    current_tilt_position = tilt_position_up;

    res.ack = 1;
    return(true);
} 

int main(int argc, char** argv)
{
    sleep(3);

    ros::init(argc, argv, "lidar");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    double v;
    int serial_number = -1;
    nh.getParam("serial", serial_number);
    pan_servo_index = 0;
    nh.getParam("panservo", pan_servo_index);
    tilt_servo_index = 1;
    nh.getParam("tiltservo", tilt_servo_index);
    range_sensor_index = 0;
    nh.getParam("sensor", range_sensor_index);
    nh.getParam("sensors", no_of_sensors);
    if (no_of_sensors < 1) no_of_sensors = 1;
    if (no_of_sensors > 10) no_of_sensors = 10;
    sensor_type.clear();
    sensor_coupling.clear();
    int sensortype = -1;
    nh.getParam("sensortype", sensortype);
    for (int s = 0; s < no_of_sensors; s++) {
        sensor_coupling.push_back(-1);
        if (sensortype > -1) {
            sensor_type.push_back(sensortype);
        }
        else {
            sensor_type.push_back(SENSOR_SHARP_GP2Y0A700K0F);
        }
    }
    nh.setParam("sensortype", -1);
    for (int s = 0; s < no_of_sensors; s++) {
        std::string str = "sensortype";
        
        char numstr[2];
        sprintf(numstr, "%d", s);
        
        str += numstr;
        int sensortype = -1;
        nh.getParam(str, sensortype);
        if (sensortype > -1) sensor_type[s] = sensortype;
        nh.setParam(str, -1);

        str = "sensorcouple";
        str += numstr;
        int sensorcouple = -1;
        nh.getParam(str, sensorcouple);
        if (sensorcouple > -1) {
			sensor_coupling[s] = sensorcouple;
		}
        nh.setParam(str, -1);
    }
    nh.getParam("x", lidar_x_m);
    nh.getParam("y", lidar_y_m);
    nh.getParam("z", lidar_z_m);
    nh.getParam("rotation", v);
	// convert to radians
    lidar_rotation_radians = v / 180.0f * 3.1415927f;
    nh.getParam("tilt", v);
	// convert to radians
    lidar_tilt_radians = v / 180.0f * 3.1415927f;
    nh.getParam("centredist", sensor_dist_from_centre_of_rotation_m);

    for (int s = 0; s < no_of_sensors; s++) {
        switch(sensor_type[s]) {
		case SENSOR_SHARP_GP2Y0A700K0F: {
			ROS_INFO("Sensor %d: Sharp GP2Y0A700K0F", s);
			break;
		}
		case SENSOR_SHARP_GP2Y0A2JYK: {
			ROS_INFO("Sensor %d: Sharp GP2Y0A2JYK", s);
			break;
		}
		case SENSOR_SHARP_GP2D120: {
			ROS_INFO("Sensor %d: Sharp GP2D120", s);
			break;
		}
		case SENSOR_SHARP_GP2D12: {
			ROS_INFO("Sensor %d: Sharp GP2D12", s);
			break;
		}
		case SENSOR_SHARP_GP2Y0A02YK: {
			ROS_INFO("Sensor %d: Sharp GP2Y0A02YK", s);
			break;
		}
		}
    }

    pan_position_left = 127 - 50;
    pan_position_left_angle_degrees = -45;
    pan_position_right = 127 + 50;
    pan_position_right_angle_degrees = 45;
    nh.getParam("speed", v);
    if ((int)v != 0) speed = (float)v;
    nh.getParam("acceleration", v);
    if ((int)v != 0) acceleration = (float)v;

    nh.getParam("left", v);
    if ((int)v != 0) pan_position_left = (float)v;
    nh.getParam("leftdegrees", v);
    if ((int)v != 0) {
		pan_position_left_angle_degrees = (float)v;
	}
    nh.getParam("right", v);
    if ((int)v != 0) pan_position_right = (float)v;
    nh.getParam("rightdegrees", v);
    if ((int)v != 0) {
		pan_position_right_angle_degrees = (float)v;
	}

    nh.getParam("up", v);
    if ((int)v != 0) tilt_position_up = (float)v;
    nh.getParam("updegrees", v);
    if ((int)v != 0) tilt_position_up_angle_degrees = (float)v;
    nh.getParam("down", v);
    if ((int)v != 0) tilt_position_down = (float)v;
    nh.getParam("downdegrees", v);
    if ((int)v != 0) {
		tilt_position_down_angle_degrees = (float)v;
	}
    nh.getParam("tiltsteps", no_of_tilt_steps);

    nh.getParam("cycletime", v);
    if ((int)v != 0) cycle_time_sec = (float)v;
    if (cycle_time_sec < 2) cycle_time_sec = 2;

    const int buffer_length = 100;        
    std::string topic_name = "phidgets/lidar";
    if (serial_number > -1) {
        char ser[10];            
        sprintf(ser,"%d", serial_number);
        topic_name += "/";
        topic_name += ser;
    }
    lidar_pub =
		n.advertise<sensor_msgs::LaserScan>(topic_name,
											buffer_length);
    ros::Subscriber servo_sub =
		n.subscribe("phidgets/servos", 1, servoCallback);
    client_servo_reference =
		n.serviceClient<phidgets::servo_reference>("servo_reference");
    ros::Subscriber interface_kit_sub =
		n.subscribe("phidgets/interface_kit",
					1, interfaceKitCallback);

    // start service which can be used to change
	// lidar parameters
    ros::ServiceServer service_lidar =
		n.advertiseService("phidgets/lidar_control",
						   change_lidar_params);

    if (paused) {
        ROS_INFO("LIDAR paused");
        // deactive the servo
        set_servo_reference(pan_servo_index, false, 0, 0, 0);
    }

    bool move_complete;
    tf::TransformBroadcaster transform_broadcaster;
    ros::Time start_time;
    ros::Rate loop_rate(30);
    suppress = true;
    while(n.ok()) {

        if (paused) {
            ros::spinOnce();
            loop_rate.sleep();
            if (!suppress) {                
                suppress = true;
                // deactive the servo
                set_servo_reference(pan_servo_index,
									false, 0, 0, 0);
                set_servo_reference(tilt_servo_index,
									false, 0, 0, 0);
            }
            publish_transforms(transform_broadcaster);
        }
        else {
            for (int i = 0; i < 2; i++) {
                if (no_of_tilt_steps > 0) {
                    set_servo_reference(tilt_servo_index,
										true,
										current_tilt_position,
										speed, acceleration);
                    sleep(1);
                }
                set_servo_reference(pan_servo_index, true,
									pan_position_left,
									speed, acceleration);
                start_time = ros::Time::now();
                move_complete = false;
                while ((n.ok()) && (!move_complete)) {

                    // publish scans one at a time
                    if (publish_scan_index > 0) {
                        lidar_pub.publish(scan[no_of_sensors -
											   1 -
											   (publish_scan_index-1)]);
                        publish_scan_index--;
                    }

                    publish_transforms(transform_broadcaster);

                    ros::spinOnce();
                    loop_rate.sleep();
                    double time_elapsed_sec =
						(ros::Time::now() -
						 start_time).toSec();
                    if (time_elapsed_sec > cycle_time_sec) {
                        move_complete = true;
                    }
                }
  
                // swap directions
                float temp = pan_position_left;
                pan_position_left = pan_position_right;
                pan_position_right = temp;
                temp = pan_position_left_angle_degrees;
                pan_position_left_angle_degrees =
					pan_position_right_angle_degrees;
                pan_position_right_angle_degrees = temp;
                suppress = false;

                if (no_of_tilt_steps > 0) {
                    // set the tilt position
                    current_tilt_position =
						tilt_position_up +
						((tilt_position_down -
						  tilt_position_up)*tilt_step /
						 no_of_tilt_steps);
                    float lidar_tilt_degrees =
						tilt_position_up_angle_degrees +
						((tilt_position_down_angle_degrees -
						  tilt_position_up_angle_degrees)*
						 tilt_step/no_of_tilt_steps);
                    lidar_tilt_radians =
						lidar_tilt_degrees * 3.1415927f /
						180.0f;
                    if (tilt_step_direction == 1) {
                        tilt_step++;
                        if (tilt_step >= no_of_tilt_steps) {
							tilt_step_direction = -1;
						}
                    }
                    else {
                        tilt_step--;
                        if (tilt_step <= 0) {
							tilt_step_direction = 1;
						}
                    }
                }
                
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
    }

    // deactivate the servo
    set_servo_reference(pan_servo_index, false, 0, 0, 0);

    return 0;
}

