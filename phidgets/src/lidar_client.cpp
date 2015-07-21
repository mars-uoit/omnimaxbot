/*
  Example ROS subscription to phidgets based LIDAR
  Copyright (C) 2010 Bob Mottram
  fuzzgun@gmail.com

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include "phidgets/lidar_control.h"

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <string.h>

#undef USE_OPENCV
#ifdef USE_OPENCV
#include <cv.h>
#include <highgui.h>
#endif

enum {
	SENSOR_SHARP_GP2Y0A700K0F = 0,
	SENSOR_SHARP_GP2Y0A2JYK,
	SENSOR_SHARP_GP2D120,
	SENSOR_SHARP_GP2D12,
	SENSOR_SHARP_GP2Y0A02YK
};

using namespace std;

#ifdef USE_OPENCV
IplImage* lidar_image = NULL;
#else
unsigned char * lidar_image = NULL;
#endif
int lidar_image_width = 640;
int lidar_image_height = 640;
int lidar_image_dimension_mm = 5500;
bool show_image = false;
int no_of_sensors = 4;
int no_of_coupled_sensors = 1;
bool scan_received = false;

ros::ServiceClient client_lidar_control;
tf::StampedTransform lidar_transform;

/*!
 * \brief callback when a scan is received
 */
void lidarCallback(const sensor_msgs::LaserScanConstPtr& ptr)
{
    sensor_msgs::LaserScan scan = *ptr;
    ROS_INFO("Received scan %s", scan.header.frame_id.c_str());

    if ((lidar_image != NULL) && (scan.header.frame_id != "")) {
        std::string sensor_index_str =
			scan.header.frame_id.substr(scan.header.frame_id.length()-1,1);
        int sensor_index = atoi(sensor_index_str.c_str());
        if (sensor_index < no_of_sensors) {
        
            int i,j;
#ifdef USE_OPENCV
            unsigned char * lidar_image_ =
				(unsigned char *)lidar_image->imageData;
#else
            unsigned char * lidar_image_ = lidar_image;
#endif
            if (sensor_index == 0) {
                for (i = lidar_image_width*lidar_image_height*3-1;
					 i >= 0; i--) {
					lidar_image_[i] = 255;
				}
            }

            int increments = (int)scan.ranges.size();
            float ang =
				(sensor_index * 2 * 3.1415927f /
				 no_of_sensors) + scan.angle_min;
            int cx = lidar_image_width/2;
            int cy = lidar_image_height/2;
            float mult =
				1.0f / (lidar_image_dimension_mm * 2 / 1000.0f);
            for (i = 0; i < increments;
				 i++, ang += scan.angle_increment) {
                float range_metres = scan.ranges[i];
                int dx =
					(int)(cx * range_metres * sin(ang) * mult);
                int dy =
					(int)(cy * range_metres * cos(ang) * mult);
                int x = cx + dx;
                int y = cy - dy;
                if ((x > 0) && (x < lidar_image_width-1) &&
                    (y > 0) && (y < lidar_image_height-1)) {
                    int length = (int)sqrt(dx*dx + dy*dy);
                    for (j = 0; j < length; j++) {
                        int xx = cx + (dx*j/length);
                        int yy = cy - (dy*j/length);
                        int n = (yy*lidar_image_width + xx)*3;
                        lidar_image_[n] = 0;
                        lidar_image_[n+1] = 255;
                        lidar_image_[n+2] = 255;
                    }
                    for (int yy = y-1; yy <= y+1; yy++) {
                        int n = (yy*lidar_image_width + x-1)*3;
                        for (int xx = x-1; xx <= x+1; xx++, n += 3) {                        
                            lidar_image_[n] = 0;
                            lidar_image_[n+1] = 0;
                            lidar_image_[n+2] = 0;
                        }  
                    }
                }
            }
            if (sensor_index == no_of_sensors-1) {
                show_image = true;
            }
            scan_received = true;
        }
    }
}


/*!
 * \brief set the parameters used for LIDAR
 * \param pause whether to pause the LIDAR
 * \param pan_servo_index index number of the pan servo on the advanced servo controller
 * \param tilt_servo_index index number of the tilt servo on the advanced servo controller
 * \param range_sensor_index index number of the sensor on the interface kit
 * \param no_of_sensors the number of range sensors
 * \param sensor_type the type of range sensors used
 * \param sensor_coupling coupling between sensors
 * \param speed speed of movement of the servo
 * \param acceleration acceleration of the servo
 * \param pan_position_left leftmost position of the pan servo in servo coordinates
 * \param pan_position_right rightmost position of the pan servo in servo coordinates
 * \param pan_position_left_angle_degrees leftmost position of the pan servo in degrees
 * \param pan_position_right_angle_degrees rightmost position of the pan servo in degrees
 * \param tilt_position_up up position of the tilt servo in servo coordinates
 * \param tilt_position_down down position of the tilt servo in servo coordinates
 * \param tilt_position_up_angle_degrees up position of the tilt servo in degrees
 * \param tilt_position_down_angle_degrees down position of the tilt servo in degrees
 * \param tilt_steps the number of tilt steps (set to zero if tilt axis is not used)
 * \param cycle_time_sec cycle time for the scan in seconds
 */
void set_lidar_params(bool pause,
					  int pan_servo_index,
					  int tilt_servo_index,
					  int range_sensor_index,
					  int no_of_sensors,
					  vector<int> &sensor_type,
					  vector<int> &sensor_coupling,
					  float speed,
					  float acceleration,
					  float pan_position_left,
					  float pan_position_right,
					  float pan_position_left_angle_degrees,
					  float pan_position_right_angle_degrees,
					  float tilt_position_up,
					  float tilt_position_down,
					  float tilt_position_up_angle_degrees,
					  float tilt_position_down_angle_degrees,
					  int tilt_steps,
					  float cycle_time_sec)
{
    phidgets::lidar_control srv;
    srv.request.pan_servo_index = pan_servo_index;
    srv.request.tilt_servo_index = tilt_servo_index;
    srv.request.sensor_index = range_sensor_index;
    srv.request.no_of_sensors = no_of_sensors;
    srv.request.sensor_type = sensor_type;
    srv.request.sensor_coupling = sensor_coupling;
    srv.request.speed = speed;
    srv.request.acceleration = acceleration;
    srv.request.pan_position_left = pan_position_left;
    srv.request.pan_position_right = pan_position_right;
    srv.request.pan_position_left_angle_degrees =
		pan_position_left_angle_degrees;
    srv.request.pan_position_right_angle_degrees =
		pan_position_right_angle_degrees;
    srv.request.tilt_position_up = tilt_position_up;
    srv.request.tilt_position_down = tilt_position_down;
    srv.request.tilt_position_up_angle_degrees =
		tilt_position_up_angle_degrees;
    srv.request.tilt_position_down_angle_degrees =
		tilt_position_down_angle_degrees;
    srv.request.tilt_steps = tilt_steps;
    srv.request.cycle_time_sec = cycle_time_sec;

    if (client_lidar_control.call(srv)) {
        ROS_INFO("Changed LIDAR parameters");
    }
    else {
        ROS_ERROR("Failed to call service lidar_control");
    }
}

int main(int argc, char** argv)
{
#ifdef USE_OPENCV	
    lidar_image =
		cvCreateImage(cvSize(lidar_image_width,
							 lidar_image_height), 8, 3);
    cvNamedWindow("LIDAR", CV_WINDOW_AUTOSIZE);
#else
	lidar_image =
		new unsigned char[lidar_image_width*
						  lidar_image_height*3];
#endif

    ros::init(argc, argv, "lidar_client");
    ros::NodeHandle n;
    ros::Subscriber lidar_sub =
		n.subscribe("phidgets/lidar", 1, lidarCallback);
    client_lidar_control =
		n.serviceClient<phidgets::lidar_control>("phidgets/lidar_control");
    tf::TransformListener listener;

    bool pause = false;
    int pan_servo_index = 0;
    int tilt_servo_index = 1;
    int range_sensor_index = 3;
    no_of_sensors = 4;
    no_of_coupled_sensors = 1;
    vector<int> sensor_type;
    vector<int> sensor_coupling;
    // long range sensors
    for (int s = 0; s < no_of_sensors; s++) {
        sensor_type.push_back(SENSOR_SHARP_GP2Y0A700K0F);
        if (s < no_of_coupled_sensors) {
            sensor_coupling.push_back(s + no_of_sensors);
        }
        else {
            sensor_coupling.push_back(-1);
        }
    }
    // short range sensors used for disambiguation
    for (int s = no_of_sensors;
		 s < no_of_sensors + no_of_coupled_sensors; s++) {
        sensor_type.push_back(SENSOR_SHARP_GP2Y0A2JYK);
        sensor_coupling.push_back(-1);
    }
    float speed = 30;
    float acceleration = 100;
    float pan_position_left = 127-50;
    float pan_position_right = 127+50;
    float pan_position_left_angle_degrees = -45;
    float pan_position_right_angle_degrees = 45;
    float tilt_position_up = 127;
    float tilt_position_down = 127-40;
    float tilt_position_up_angle_degrees = 0;
    float tilt_position_down_angle_degrees = -45;
    int tilt_steps = 5;
    float cycle_time_sec = 4;

    set_lidar_params(pause,
					 pan_servo_index,
					 tilt_servo_index,
					 range_sensor_index,
					 no_of_sensors + no_of_coupled_sensors,
					 sensor_type,
					 sensor_coupling,
					 speed,
					 acceleration,
					 pan_position_left,
					 pan_position_right,
					 pan_position_left_angle_degrees,
					 pan_position_right_angle_degrees,
					 tilt_position_up,
					 tilt_position_down,
					 tilt_position_up_angle_degrees,
					 tilt_position_down_angle_degrees,
					 tilt_steps,
					 cycle_time_sec);

    scan_received = false;
    ros::Rate loop_rate(30);
    while(n.ok()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (scan_received) {
            // get the latest available transform
            try {                
                listener.lookupTransform("/base_link",
										 "/base_lidar",
										 ros::Time(0),
										 lidar_transform);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }
            tf::Quaternion rot = lidar_transform.getRotation();
        }

        if (show_image) {
#ifdef USE_OPENCV
            cvShowImage("LIDAR", lidar_image);
#else
			// TODO: save the image to file every few seconds
#endif
            show_image = false;
        }
#ifdef USE_OPENCV
        int wait = cvWaitKey(10) & 255;
        if( wait == 27 ) break;
#endif
    }

    set_lidar_params(true,
					 pan_servo_index,
					 tilt_servo_index,
					 range_sensor_index,
					 no_of_sensors + no_of_coupled_sensors,
					 sensor_type,
					 sensor_coupling,
					 speed,
					 acceleration,
					 pan_position_left,
					 pan_position_right,
					 pan_position_left_angle_degrees,
					 pan_position_right_angle_degrees,
					 tilt_position_up,
					 tilt_position_down,
					 tilt_position_up_angle_degrees,
					 tilt_position_down_angle_degrees,
					 tilt_steps,        
					 cycle_time_sec);

#ifdef USE_OPENCV
    cvReleaseImage(&lidar_image);
#else
	delete[] lidar_image;
#endif

    return 0;
}

