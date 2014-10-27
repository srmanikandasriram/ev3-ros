/*
* Software License Agreement (MIT License)
*
* Copyright (c) 2014 S.R.Manikandasriram
* All rights reserved.
*
* This software is distributed under MIT License.
*/

#include <ros.h>
#include <std_msgs/ColorRGBA.h>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>
#include "ev3dev.h"

using namespace std;
using namespace ev3dev;

ros::NodeHandle  nh;

// Construct a unique name
basic_string<char> name;
sensor s;

int main(int argc, char* argv[])
{
	if(argc<3)
	{
		cerr << "Usage: " << argv[0] << " <socket> <sensor_port>" << endl;
		return 1;
	}
    
    nh.initNode(argv[1]);
	while(!nh.connected()) {nh.spinOnce();}
	
    // TODO: Check for valid nh and raise error if otherwise
    int sensor_port = atoi(argv[2]);
    if(sensor_port<1||sensor_port>4)
    {
		cerr << "Invalid sensor port number. Must be 1, 2, 3 or 4." << endl;
		return 1;
	}

	string port (argv[2]);
	name = "color_sensor"+port;
    s = sensor(sensor_port);

    if(s.as_string(s.type())!="EV3 color")
    {
		cerr << "Invalid sensor type. Must be EV3 color. Given sensor is of type " << s.as_string(s.type()) << endl;
		return 1;
	}    	

	// Set mode
	string mode="COL-AMBIENT";
	s.set_mode(mode);

 	std_msgs::ColorRGBA color_msg;
	color_msg.a = 1;

 	ros::Publisher color_pub("range", &color_msg);
 	nh.advertise(color_pub);
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	// TODO: Test for frequency compliance and implementation of ros::Rate
	while(1){

		nh.spinOnce();               // check for incoming messages
		current_time = ros::Time::now();

		color_msg.r = s.value();
		color_msg.g = s.value();
		color_msg.b = s.value();
		color_pub.publish(&color_msg);

		last_time = current_time;
		sleep(1.0);

	}
    return 0;

}