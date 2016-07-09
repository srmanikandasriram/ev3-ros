/*
* Software License Agreement (MIT License)
*
* Copyright (c) 2014 S.R.Manikandasriram
* All rights reserved.
*
* This software is distributed under MIT License.
*/

#include <ros.h>
#include <sensor_msgs/Range.h>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>
#include "ev3dev.h"

using namespace std;
using namespace ev3dev;

ros::NodeHandle  nh;
char* ip;

float deg2rad = M_PI/180.0;

// Construct a unique name
//basic_string<char> name;
sensor s = sensor(INPUT_4);

int main(int argc, char* argv[])
{
	if(argc<3)
	{
		std::cerr << "Usage: " << argv[0] << " <IP> <sensor_port>" << std::endl;
		return 1;
	}
     ip =  (argv[1]);
    nh.initNode(ip);

    while(!nh.connected()) {nh.spinOnce();}

    // TODO: Check for valid nh and raise error if otherwise
    string sensor_port = (argv[2]);
    //if(sensor_port<1||sensor_port>4)
   // {
	//	std::cerr << "Invalid sensor port number. Must be 1, 2, 3 or 4." << std::endl;
	//	return 1;
	//}


	//name = "sensor"+sensor_port;
    s = sensor(sensor_port);

   // if(s.as_string(s.type())!="EV3 ultrasonic")
  //  {
		//std::cerr << "Invalid sensor type. Must be EV3 ultrasonic. Given sensor is of type " << s.as_string(s.type()) << std::endl;
		//return 1;
	//}    	

	// Set mode
	string mode="US-DIST-CM";
	s.set_mode(mode);

 	sensor_msgs::Range us_msg;
 	us_msg.header.frame_id = "pi-alpha";
	us_msg.radiation_type = 0;
	us_msg.field_of_view = 5*deg2rad; // Approximating to 5deg FOV
	us_msg.min_range = 0.03;
	us_msg.max_range = 2.55;

 	ros::Publisher us_pub("range", &us_msg);
 	nh.advertise(us_pub);
	
	ros::Time current_time, last_time;
	current_time = nh.now();
	last_time = nh.now();

	// TODO: Test for frequency compliance and implementation of ros::Rate
	while(1){

		nh.spinOnce();               // check for incoming messages
		current_time = nh.now();

		us_msg.header.stamp = current_time;
		us_msg.range = s.value()/1000.0;
		cout<<s.value()/1000.0<<endl;
		us_pub.publish(&us_msg);

		last_time = current_time;
		sleep(1.0);

	}
    return 0;

}