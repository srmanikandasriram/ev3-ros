/*
* Software License Agreement (MIT License)
*
* Copyright (c) 2014 S.R.Manikandasriram
* All rights reserved.
*
* This software is distributed under MIT License.
*/

#include <ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>
#include "ev3dev.h"
#include <thread>
#include <chrono>

using namespace std;
using namespace ev3dev;

string ns = "/robot2/";
ros::NodeHandle  nh;

float x = 0.0, y = 0.0, t = 0.0;
float x_est = 0.0, y_est = 0.0, t_est = 0.0;
bool new_estimate = false;
float R, L, speed;	// Update R and L from Parameter server
const float deg2rad = M_PI/180.0;

motor left_motor, right_motor;
sensor s;

float vx, wt, vl, vr;
bool running = true;

int encoder[2]={0,0}, prev_encoder[2] = {0,0}, dl, dr;

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
	cout << "received " << vx << "," << wt << "=>" << vl <<"," << vr << endl;
}

void pose_cb(const geometry_msgs::PoseStamped& msg){
	x_est = msg.pose.position.x;
	y_est = msg.pose.position.y;
	geometry_msgs::Quaternion q = msg.pose.orientation;
	t_est = 2.0*atan2(q.z,q.w);
	new_estimate = true;
	cout << " Estimate: " << x_est << "," << y_est << "," << t_est <<endl;
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("/robot2/cmd_vel", cmd_vel_cb );
ros::Subscriber<geometry_msgs::PoseStamped> pose_sub("pose_estimate_1" , pose_cb );

void odometry()
{
	int encoder[2]={0,0}, prev_encoder[2] = {0,0}, dl, dr;
	while(running){
		if(new_estimate){
			x = x_est;
			y = y_est;
			t = t_est;
			new_estimate = false;
		}
		encoder[0] = left_motor.position();
		encoder[1] = right_motor.position();
		dl = encoder[0]-prev_encoder[0];
		dr = encoder[1]-prev_encoder[1];
		x += cos(t)*speed*(dl+dr)/2.0;
		y += sin(t)*speed*(dl+dr)/2.0;
		t += speed*(dl-dr)/L;
		prev_encoder[0] = encoder[0];
		prev_encoder[1] = encoder[1];		
		vl = left_motor.pulses_per_second();
		vr = right_motor.pulses_per_second();
		vx = (vl+vr)/2*speed;
		wt = (vl-vr)/L*speed;
		this_thread::sleep_for(chrono::milliseconds(100));
	}
}

int main(int argc, char* argv[])
{
	if(argc<5)
	{
		cerr << "Usage: " << argv[0] << " <socket> <left_motor_port> <right_motor_port> <sensor_port> <hz>" << endl;
		return 1;
	}
	int milliseconds = 100;
	if(argc==6)
    	milliseconds = 1000/atoi(argv[5]);
    // cout<<"milliseconds"<<milliseconds;
    string left_motor_port (argv[2]);
    string right_motor_port (argv[3]);
    string sensor_port (argv[4]);
    // if(left_motor_port<1||left_motor_port>4||right_motor_port<1||right_motor_port>4||left_motor_port==right_motor_port)
    // {
		// cerr << "Invalid motor port numbers. Must be 1, 2, 3 or 4 and distinct." << endl;
		// return 1;
	// }

	// TODO: Check if both are of same type

	left_motor = motor(left_motor_port);
	right_motor = motor(right_motor_port);
	s = sensor(sensor_port);
    if(s.type()!="ev3-uart-30")
    {
		cerr << "Invalid sensor type. Must be EV3 ultrasonic. Given sensor is of type " << s.type() << endl;
		return 1;
	}    	

	// TODO: Check if both were initialised

	left_motor.reset();
	left_motor.set_position(0);
	left_motor.set_run_mode("forever");
	left_motor.set_stop_mode("brake");
	left_motor.set_regulation_mode("on");

	right_motor.reset();
	right_motor.set_position(0);
	right_motor.set_run_mode("forever");
	right_motor.set_stop_mode("brake");
	right_motor.set_regulation_mode("on");

	// Set mode
	string mode="US-DIST-CM";
	s.set_mode(mode);

	cout<<" Initialiased motors."<<endl;

    nav_msgs::Odometry odom_msg;
	ros::Publisher odom_pub("/robot2/odom", &odom_msg);
	nh.advertise(odom_pub);
	tf::TransformBroadcaster odom_broadcaster;
	tf::TransformBroadcaster scan_broadcaster;
	nh.subscribe(cmd_sub);
	nh.subscribe(pose_sub);
	cout<<" Initialiased Publisher and TransformBroadcaster."<<endl;

 	sensor_msgs::LaserScan us_msg;
 	ros::Publisher us_pub("/robot2/scan", &us_msg);
 	nh.advertise(us_pub);
 	int number = 100;
 	float ranges[100]={0.0}, intensities[100] = {255};
 	us_msg.header.frame_id = "laser_frame";
	us_msg.angle_min = -M_PI/40;
    us_msg.angle_max = M_PI/40;
    us_msg.angle_increment = M_PI/(20*number);
    us_msg.time_increment = 0.0000;
    us_msg.scan_time = 0.0000;
    us_msg.range_min = 0.01;
    us_msg.range_max = 2.55;
    us_msg.ranges_length = number;
    us_msg.intensities_length = number;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	//ros::Rate r(1.0);
    
    cout<<" Initialiased Time variables."<<endl;
    
    nh.initNode(argv[1]);
	odom_broadcaster.init(nh);
	scan_broadcaster.init(nh);
    while(!nh.connected()) {nh.spinOnce();}

	cout<<" Initialiased node."<<endl;
	
	if (! nh.getParam((ns+"R").c_str(), &R)){ 
	     R = 0.03;
	     speed = R*deg2rad;
	}

	if (! nh.getParam((ns+"L").c_str(), &L)){ 
	     L = 0.12;
	}
	
	cout<<" Retrieved params R,L : "<<R<<","<<L<<endl;
	thread odom (odometry);
	while(1){

		nh.spinOnce();               // check for incoming messages
		current_time = ros::Time::now();

		// encoder[0] = left_motor.position();
		// encoder[1] = right_motor.position();
		// dl = encoder[0]-prev_encoder[0];
		// dr = encoder[1]-prev_encoder[1];
		// x += cos(t)*speed*(dl+dr)/2.0;
		// y += sin(t)*speed*(dl+dr)/2.0;
		// t += speed*(dl-dr)/L;
		// prev_encoder[0] = encoder[0];
		// prev_encoder[1] = encoder[1];
	
		// cout<<" Calculated: "<<x<<","<<y<<","<<t<<endl;

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(t);

		// cout<<" Calculated quat: "<<endl;

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "map";
		odom_trans.child_frame_id = "base_link";

		// cout<<" constructed header"<<endl;

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		// cout<<" Constructed full message"<<endl;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		geometry_msgs::Quaternion scan_quat = tf::createQuaternionFromYaw(0);

		geometry_msgs::TransformStamped scan_trans;
		scan_trans.header.stamp = current_time;
		scan_trans.header.frame_id = "base_link";
		scan_trans.child_frame_id = "laser_frame";

		// cout<<" constructed header"<<endl;

		scan_trans.transform.translation.x = L/2.0;
		scan_trans.transform.translation.y = 0;
		scan_trans.transform.translation.z = 0;
		scan_trans.transform.rotation = scan_quat;

		// cout<<" Constructed full message"<<endl;

		//send the transform
		scan_broadcaster.sendTransform(scan_trans);

		// cout<<" Sent transform"<<endl;

		//next, we'll publish the odometry message over ROS
		odom_msg.header.stamp = current_time;
		odom_msg.header.frame_id = "map";

		//set the position
		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.position.z = 0.0;
		odom_msg.pose.pose.orientation = odom_quat;

		//set the velocity
		odom_msg.child_frame_id = "base_link";
		odom_msg.twist.twist.linear.x = vx;
		odom_msg.twist.twist.linear.y = 0;
		odom_msg.twist.twist.angular.z = wt;

		// cout<<" sizeof "<<sizeof(odom_msg)<<endl;
		//publish the message
		odom_pub.publish(&odom_msg);

		// cout<<" Published odometry"<<endl;
		us_msg.header.stamp = current_time;
		float distance = s.value()/1000.0;
	    for (int i = 0; i < number; ++i)
	    {
	    	ranges[i] = distance;
	    	// if(abs(i-number/2)<4)
	    		// ranges[i] = distance;
	    }
	    us_msg.ranges = ranges;
	    us_msg.intensities = intensities;
	    us_pub.publish(&us_msg);
		cout<<" sizeof "<<sizeof(us_msg)<<endl;
		last_time = current_time;
		this_thread::sleep_for(chrono::milliseconds(milliseconds));
	}
	odom.join();
    return 0;

}
