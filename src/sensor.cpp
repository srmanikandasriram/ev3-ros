/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014 S.R.Manikandasriram
* All rights reserved.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros.h>
#include <sensor_msgs/Range.h>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>
#include "ev3dev.h"

ros::NodeHandle  nh;

// Construct a unique name
char *a[];
sensor s;

void jc_cb(const ev3_msgs::JointCommand& msg){
	if(strcmp(msg.name,name)==0)
	{
		cmd = msg.effort/POWER_TO_NM;
        if(cmd > POWER_MAX)
			cmd = POWER_MAX
        else if(cmd < -POWER_MAX)
			cmd = -POWER_MAX
    }
    joint.set_pulses_per_second_setpoint(cmd);
}

ros::Subscriber<ev3_msgs::JointCommand> jc_sub("joint_command", jc_cb );

int main(int argc, char* argv[])
{
	if(argc<3)
	{
		std::cerr << "Usage: " << argv[0] << " <socket> <motor_port>" << std::endl;
		return 1;
	}
    
    nh.initNode(argv[1]);

    // TODO: Check for valid nh and raise error if otherwise
    int motor_port = argv[2];
    if(motor_port<1||motor_port>4)
    {
		std::cerr << "Invalid motor port number. Must be 1, 2, 3 or 4." << std::endl;
		return 1;
	}

    joint = motor(motor_port);

    if(strcmp(joint.type(),"minitacho")==0)
    	POWER_TO_NM = 0.08;

    // Initialise motor
	joint.reset();
	joint.set_position(0);
	joint.set_run_mode("forever");
	joint.set_stop_mode("brake");
	joint.set_regulation_mode("on");

 	sensor_msgs::JointState js_msg;
 	ros::Publisher js_pub("joint_state", &js_msg);
 	nh.advertise(js_pub);

	nh.subscribe(jc_sub);
 	
 	// nav_msgs::Odometry odom_msg;
	// ros::Publisher odom_pub("odom", &odom_msg);
	// nh.advertise(odom_pub);
	// tf::TransformBroadcaster odom_broadcaster;
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	// TODO: Test for frequency compliance and implementation of ros::Rate
	//ros::Rate r(1.0);
	while(ros::ok()){

		nh.spinOnce();               // check for incoming messages
		current_time = ros::Time::now();

		js_msg.header.stamp = current_time;
		// strcpy(js_msg.header.frame_id,"joint_frame");
		js_msg.name.push_back(name);
		
		float position = joint.position()*deg2rad, velocity = joint.pulses_per_second()*deg2rad, power = joint.duty_cycle()*POWER_TO_NM;
		js_msg.position.push_back(position);
		js_msg.velocity.push_back(velocity);
		js_msg.effort.push_back(power);
		js_pub.publish(&js_msg);

		last_time = current_time;
		sleep(1.0);

		// //compute odometry in a typical way given the velocities of the robot
		// double dt = (current_time - last_time).toSec();
		// double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		// double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		// double delta_th = vth * dt;

		// x += delta_x;
		// y += delta_y;
		// th += delta_th;

		// //since all odometry is 6DOF we'll need a quaternion created from yaw
		// geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);

		// //first, we'll publish the transform over tf
		// geometry_msgs::TransformStamped odom_trans;
		// odom_trans.header.stamp = current_time;
		// odom_trans.header.frame_id = "odom";
		// odom_trans.child_frame_id = "base_link";

		// odom_trans.transform.translation.x = x;
		// odom_trans.transform.translation.y = y;
		// odom_trans.transform.translation.z = 0.0;
		// odom_trans.transform.rotation = odom_quat;

		// //send the transform
		// odom_broadcaster.sendTransform(odom_trans);

		// //next, we'll publish the odometry message over ROS
		// odom_msg.header.stamp = current_time;
		// odom_msg.header.frame_id = "odom";

		// //set the position
		// odom_msg.pose.pose.position.x = x;
		// odom_msg.pose.pose.position.y = y;
		// odom_msg.pose.pose.position.z = 0.0;
		// odom_msg.pose.pose.orientation = odom_quat;

		// //set the velocity
		// odom_msg.child_frame_id = "base_link";
		// odom_msg.twist.twist.linear.x = vx;
		// odom_msg.twist.twist.linear.y = vy;
		// odom_msg.twist.twist.angular.z = vth;

		// //publish the message
		// odom_pub.publish(&odom_msg);
	}
    return 0;

}