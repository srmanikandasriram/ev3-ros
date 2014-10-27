/*
* Software License Agreement (MIT License)
*
* Copyright (c) 2014 S.R.Manikandasriram
* All rights reserved.
*
* This software is distributed under MIT License.
*/

#include <ros.h>
#include <control_msgs/GripperCommand.h>
#include <sensor_msgs/JointState.h>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>
#include "ev3dev.h"

using namespace std;
using namespace ev3dev;

ros::NodeHandle  nh;

// From LEGO Website.
// Large Motor - Running Torque: 0.2Nm, Stalling torque: 0.4Nm
// Medium Motor - Running Torque: 0.08Nm, Stalling torque: 0.12Nm
float POWER_TO_NM  = 0.20;
#define POWER_MAX 1000
float deg2rad = M_PI/180.0;

// Construct a unique name
basic_string<char> name;
motor joint;

void jc_cb(const control_msgs::GripperCommand& msg){
	int cmd;
	cmd = msg.max_effort/POWER_TO_NM;
    if(cmd > POWER_MAX){
    	cmd = POWER_MAX;
    }else if(cmd < -POWER_MAX){
		cmd = -POWER_MAX;
    }
    if(cmd!=0){
    	joint.set_pulses_per_second_setpoint(cmd);
    	joint.run(true);
    }else{
    	joint.set_pulses_per_second_setpoint(cmd);
    	joint.run(false);
    }

	// cout << "received " << msg.max_effort << "/" << POWER_TO_NM << "=" << cmd << endl;
}

ros::Subscriber<control_msgs::GripperCommand> jc_sub("jc", jc_cb );

int main(int argc, char* argv[])
{
	if(argc<3)
	{
		cerr << "Usage: " << argv[0] << " <socket> <motor_port>" << endl;
		return 1;
	}
    cout << "Enough arguments present"<< endl;
    nh.initNode(argv[1]);

    while(!nh.connected()) {nh.spinOnce();}
	
	cout << "Initialised node"<< endl;
    // TODO: Check for valid nh and raise error if otherwise
    int motor_port = atoi(argv[2]);
    if(motor_port<1||motor_port>4)
    {
		cerr << "Invalid motor port number. Must be 1, 2, 3 or 4." << endl;
		return 1;
	}
	string port (argv[2]);
	name = "motor"+port;
    joint = motor(motor_port);
	cout << "Got valid port number : " << motor_port << " name: "<< name << endl;

    if(joint.type()=="minitacho")
    	POWER_TO_NM = 0.08;

    // Initialise motor
	joint.reset();
	joint.set_position(0);
	joint.set_run_mode("forever");
	joint.set_stop_mode("brake");
	joint.set_regulation_mode("on");

 	sensor_msgs::JointState js_msg;
 	char *a[] = {"motor"};
 	float pos[1];
 	float vel[1];
 	float eff[1];
 	ros::Publisher js_pub("joint_state", &js_msg);
 	nh.advertise(js_pub);
	cout << "advertised on joint_state"<< endl;
	nh.subscribe(jc_sub);
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	js_msg.name_length = 1;
	js_msg.position_length = 1;
	js_msg.velocity_length = 1;
	js_msg.effort_length = 1;

	// TODO: Test for frequency compliance and implementation of ros::Rate
	while(1){

		nh.spinOnce();               // check for incoming messages
		current_time = ros::Time::now();
		
		// cout << "Got time " << endl;

		js_msg.header.stamp = current_time;
		js_msg.header.frame_id = "pi-alpha";
		
		// cout << "Stamped time " << endl;

		pos[0] = joint.position()*deg2rad;
		vel[0] = joint.pulses_per_second()*deg2rad;
		eff[0] = joint.duty_cycle()*POWER_TO_NM;
		// cout << "Got data" << endl;
		js_msg.name = a;
		js_msg.position = pos;
		js_msg.velocity = vel;
		js_msg.effort = eff;
		// cout << "Constructed message" << endl;
		js_pub.publish(&js_msg);
		// cout << "Published msg" << endl;
		last_time = current_time;
		sleep(1.0);
		// cout << "Finished sleep " << endl;

	}
    return 0;

}