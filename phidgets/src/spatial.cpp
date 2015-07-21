/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets Spatial sensor
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

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <phidget21.h>
#include <std_msgs/String.h>
#include "phidgets/spatial_params.h"

// Handle
CPhidgetSpatialHandle phid;

// state publisher
ros::Publisher spatial_pub;

bool initialised = false;

// axis indices
int axis_id[3];

int AttachHandler(CPhidgetHandle phid, void *userptr)
{
    int serial_number;
    const char *name;

    CPhidget_getDeviceName (phid, &name);
    CPhidget_getSerialNumber(phid, &serial_number);
    ROS_INFO("%s Serial number %d attached!",
			 name, serial_number);

    return 0;
}

int DetachHandler(CPhidgetHandle phid, void *userptr)
{
    int serial_number;
    const char *name;

    CPhidget_getDeviceName (phid, &name);
    CPhidget_getSerialNumber(phid, &serial_number);
    ROS_INFO("%s Serial number %d detached!",
			 name, serial_number);

    return 0;
}

int ErrorHandler(CPhidgetHandle phid, void *userptr,
				 int ErrorCode, const char *Description)
{
    ROS_INFO("Error handled. %d - %s", ErrorCode, Description);
    return 0;
}


// callback that will run at datarate
// data - array of spatial event data structures that holds
//        the spatial data packets that were sent in this event
// count - the number of spatial data event packets included
//         in this event
int SpatialDataHandler(CPhidgetSpatialHandle spatial,
					   void *userptr,
					   CPhidgetSpatial_SpatialEventDataHandle *data,
					   int count)
{
    if (initialised) {
        ROS_INFO("Number of Data Packets in this event: %d\n",
				 count);
        phidgets::spatial_params s;
        for (int i = 0; i < count; i++) {
            ROS_INFO("=== Data Set: %d ===\n", i);
            ROS_INFO("Acceleration> x: %6f  y: %6f  x: %6f\n",
					 data[i]->acceleration[axis_id[0]],
					 data[i]->acceleration[axis_id[1]],
					 data[i]->acceleration[axis_id[2]]);
            ROS_INFO("Angular Rate> x: %6f  y: %6f  x: %6f\n",
					 data[i]->angularRate[axis_id[0]],
					 data[i]->angularRate[axis_id[1]],
					 data[i]->angularRate[axis_id[2]]);
            ROS_INFO("Magnetic Field> x: %6f  y: %6f  x: %6f\n",
					 data[i]->magneticField[axis_id[0]],
					 data[i]->magneticField[axis_id[1]],
					 data[i]->magneticField[axis_id[2]]);
            ROS_INFO("Timestamp> seconds: %d -- microseconds: %d\n",
					 data[i]->timestamp.seconds,
					 data[i]->timestamp.microseconds);
            s.acceleration.push_back(data[i]->acceleration[axis_id[0]]);
            s.acceleration.push_back(data[i]->acceleration[axis_id[1]]);
            s.acceleration.push_back(data[i]->acceleration[axis_id[2]]);

            // tilt
            double ratio,angle=0;
            angle =
				asin(data[i]->acceleration[axis_id[1]])*
				180/3.1415927;
            s.orientation.push_back(angle);

            // roll
            angle =
				asin(data[i]->acceleration[axis_id[0]])*
				180/3.1415927;
            s.orientation.push_back(angle);

            s.angular.push_back(data[i]->angularRate[axis_id[0]]);
            s.angular.push_back(data[i]->angularRate[axis_id[1]]);
            s.angular.push_back(data[i]->angularRate[axis_id[2]]);
            s.magnetic.push_back(data[i]->magneticField[axis_id[0]]);
            s.magnetic.push_back(data[i]->magneticField[axis_id[1]]);
            s.magnetic.push_back(data[i]->magneticField[axis_id[2]]);
            s.timestamp.push_back((double)(data[i]->timestamp.seconds) +
								  (data[i]->timestamp.microseconds/1000000.0));
        }
        spatial_pub.publish(s);

        ROS_INFO("---------------------------------------------\n");
    }

    return 0;
}

int display_properties(CPhidgetSpatialHandle phid)
{
    int serialNo, version;
    const char* ptr;
    int numAccelAxes, numGyroAxes, numCompassAxes;
	int dataRateMax, dataRateMin;

    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);
    CPhidgetSpatial_getAccelerationAxisCount((CPhidgetSpatialHandle)phid,
											 &numAccelAxes);
    CPhidgetSpatial_getGyroAxisCount((CPhidgetSpatialHandle)phid,
									 &numGyroAxes);
    CPhidgetSpatial_getCompassAxisCount((CPhidgetSpatialHandle)phid,
										&numCompassAxes);
    CPhidgetSpatial_getDataRateMax((CPhidgetSpatialHandle)phid,
								   &dataRateMax);
    CPhidgetSpatial_getDataRateMin((CPhidgetSpatialHandle)phid,
								   &dataRateMin);

    ROS_INFO("%s\n", ptr);
    ROS_INFO("Serial Number: %10d\nVersion: %8d\n",
			 serialNo, version);
    ROS_INFO("Number of Accel Axes: %i\n", numAccelAxes);
    ROS_INFO("Number of Gyro Axes: %i\n", numGyroAxes);
    ROS_INFO("Number of Compass Axes: %i\n", numCompassAxes);
    ROS_INFO("datarate> Max: %d  Min: %d\n",
			 dataRateMax, dataRateMin);

    return 0;
}

bool attach(CPhidgetSpatialHandle &phid,
			int serial_number,
			int data_rate)
{
    int result;
    const char *err;

    //create the spatial object
    CPhidgetSpatial_create(&phid);

    // Set the handlers to be run when the device is
	// plugged in or opened from software, unplugged or
	// closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid,
								  AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid,
								  DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)phid,
								 ErrorHandler, NULL);

    // Registers a callback that will run according to the
	// set data rate that will return the spatial data changes
    // Requires the handle for the Spatial, the callback
	// handler function that will be called, 
    // and an arbitrary pointer that will be supplied to
	// the callback function (may be NULL)
    CPhidgetSpatial_set_OnSpatialData_Handler(phid,
											  SpatialDataHandler,
											  NULL);

    // open the spatial object for device connections
    CPhidget_open((CPhidgetHandle)phid, -1);

    // get the program to wait for a spatial device
	// to be attached
    ROS_INFO("Waiting for spatial to be attached.... \n");
    if ((result =
		 CPhidget_waitForAttachment((CPhidgetHandle)phid,
									10000))) {
        CPhidget_getErrorDescription(result, &err);
        ROS_INFO("Problem waiting for attachment: %s\n", err);
        return 0;
    }
    else {
		//Set the data rate for the spatial events
		CPhidgetSpatial_setDataRate(phid, data_rate);
        return true;
    }
}

/*!
 * \brief disconnect
 */
void disconnect(
				CPhidgetSpatialHandle &phid)
{
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_spatial");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    int serial_number = -1;
    int data_rate = 16;
    nh.getParam("serial", serial_number);
    nh.getParam("rate", data_rate);
    std::string name = "spatial";
    nh.getParam("name", name);
    if (serial_number==-1) {
        nh.getParam("serial_number", serial_number);
    }
    std::string topic_path = "phidgets/";
    nh.getParam("topic_path", topic_path);

    int frequency = 30;
    nh.getParam("frequency", frequency);

    int ax_id = 0;
    nh.getParam("x_axis_id", ax_id);
    axis_id[0] = ax_id;

    ax_id = 1;
    nh.getParam("y_axis_id", ax_id);
    axis_id[1] = ax_id;

    ax_id = 2;
    nh.getParam("z_axis_id", ax_id);
    axis_id[2] = ax_id;

    if (attach(phid, serial_number, data_rate)) {
		display_properties(phid);

        const int buffer_length = 100;        
        std::string topic_name = topic_path + name;
        if (serial_number > -1) {
            char ser[10];            
            sprintf(ser,"%d", serial_number);
            topic_name += "/";
            topic_name += ser;
        }
        spatial_pub =
			n.advertise<phidgets::spatial_params>(topic_name,
												  buffer_length);
        ros::Rate loop_rate(frequency);

        initialised = true;

        while (ros::ok())
			{
				ros::spinOnce();
				loop_rate.sleep();
			}

        disconnect(phid);
    }
    return 0;
}

