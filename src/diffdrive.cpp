/*
* Software License Agreement (MIT License)
*
* Copyright (c) 2014 S.R.Manikandasriram
* All rights reserved.
*
* This software is distributed under MIT License.
*/

#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>
#include "ev3dev.h"
#include <thread>

using namespace std;
using namespace ev3dev;

ros::NodeHandle  nh;

float x = 0.0, y = 0.0, t = 0.0;
float R = 0.03, L = 0.12;	// Update R and L from Parameter server
const float deg2rad = M_PI/180.0, speed = R*deg2rad;
motor left_motor, right_motor;

void cmd_vel_cb(const geometry_msgs::Twist& cmd){
	float vx, wt, vl, vr;
	vx = cmd.linear.x;
	wt = cmd.angular.z;
	vl = (vx+L/2*wt)/speed;
	vr = (vx-L/2*wt)/speed;
	if(vl!=0)
	{
		left_motor.set_pulses_per_second_setpoint(vl);
		left_motor.run(true);
	}
	else
	{
		left_motor.set_pulses_per_second_setpoint(0);
		left_motor.run(false);	
	}
	if(vr!=0)
	{
		right_motor.set_pulses_per_second_setpoint(vr);
		right_motor.run(true);
	}
	else
	{
		right_motor.set_pulses_per_second_setpoint(0);
		right_motor.run(false);	
	}
	cout << "received " << vx << "," << wt << "=>" << vl <<"," << vr << std::endl;
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", cmd_vel_cb );

int main(int argc, char* argv[])
{
	if(argc<4)
	{
		std::cerr << "Usage: " << argv[0] << " <socket> <left_motor_port> <right_motor_port>" << std::endl;
		return 1;
	}
    
    nh.initNode(argv[1]);

    int left_motor_port = atoi(argv[2]), right_motor_port = atoi(argv[3]);
    if(left_motor_port<1||left_motor_port>4||right_motor_port<1||right_motor_port>4||left_motor_port==right_motor_port)
    {
		std::cerr << "Invalid motor port numbers. Must be 1, 2, 3 or 4 and distinct." << std::endl;
		return 1;
	}

	// TODO: Check if both are of same type

	left_motor = motor(left_motor_port);
	right_motor = motor(right_motor_port);

    nav_msgs::Odometry odom_msg;
	ros::Publisher odom_pub("odom", &odom_msg);
	nh.advertise(odom_pub);
	tf::TransformBroadcaster odom_broadcaster;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	float vx, wt, vl, vr;
	int encoder[2]={0,0}, prev_encoder[2] = {0,0}, dl, dr;
	//ros::Rate r(1.0);
	while(1){

		nh.spinOnce();               // check for incoming messages
		current_time = ros::Time::now();

		encoder[0] = left_motor.position();
		encoder[1] = right_motor.position();
		dl = encoder[0]-prev_encoder[0];
		dr = encoder[1]-prev_encoder[1];
		x += cos(t)*speed*(dl+dr)/2;
		y += sin(t)*speed*(dl+dr)/2;
		t += speed*(dl-dr)/L;
		prev_encoder[0] = encoder[0];
		prev_encoder[1] = encoder[1];

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(t);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		odom_msg.header.stamp = current_time;
		odom_msg.header.frame_id = "odom";

		//set the position
		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.position.z = 0.0;
		odom_msg.pose.pose.orientation = odom_quat;

		vl = left_motor.pulses_per_second();
		vr = right_motor.pulses_per_second();
		vx = (vl+vr)/2*speed;
		wt = (vl-vr)/L*speed;

		//set the velocity
		odom_msg.child_frame_id = "base_link";
		odom_msg.twist.twist.linear.x = vx;
		odom_msg.twist.twist.linear.y = 0;
		odom_msg.twist.twist.angular.z = wt;

		//publish the message
		odom_pub.publish(&odom_msg);

		last_time = current_time;
		sleep(1.0);
	}
    return 0;

}