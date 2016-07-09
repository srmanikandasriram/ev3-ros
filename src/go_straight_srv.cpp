/*The srv file for this program is of the form 
float64 vel
int64 time
---
float64 vl
float64 vr


This program enables the robot to traverse in a staight line with velocity vel m/s for time,
 time secs passed as request in the rosservice call 
*/

#include <ros.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include "ev3dev.h"
#include <chrono>
#include <headers/straight.h>
#include <stdio.h>
//I should create an srv file and add the header file here
using namespace std;
using namespace ev3dev;
using headers::straight;
ros::NodeHandle  nh; 
char* srvrIP;
motor left_motor = OUTPUT_B, right_motor = OUTPUT_C;
sensor s = INPUT_1;
float ti; 
/*the srv file must be of the form 

float vel
int time
---
float vl
float vr
*/
float R=0.03,speed; 
const float deg2rad = M_PI/180.0;
void svcCallback(const headers::straight::Request &req , headers::straight::Response &res){ // The callback function
  //float vx, wt, vl, vr; // LOCAL VARIABLES DEFINED AGAIn
  speed = R*deg2rad;
  ti = req.time*1000;//to convert secs to millisecs 
  res.vl=req.vel/speed; // /speed was taken out
  res.vr=req.vel/speed;   
  if(res.vl!=0)
  {
    left_motor.set_speed_sp(res.vl);
    left_motor.set_time_sp(ti);// time is in milliseconds
    left_motor.set_command("run-timed");// changed from forever mode to time
  }
  else
  {
    left_motor.set_speed_sp(0);
    left_motor.set_command("run-forever");
  }
  if(res.vr!=0)
  {
    right_motor.set_speed_sp(res.vr); 
    right_motor.set_time_sp(ti);
    right_motor.set_command("run-timed");// changed from forever mode to time
  }
  else
  {
    right_motor.set_speed_sp(0);
    right_motor.set_command("run-forever");
  }
  //cout << "received " << vx << "," << wt << "=>" << vl <<"," << vr << endl;
}

ros::ServiceServer<headers::straight::Request, headers::straight::Response> server("straight_srv",&svcCallback); //SERVICE DEFINITION 




int main(int argc, char* argv[]) 
{  

// These following lines are used to Make sure that command lline args are correct
  

if(argc<5) // if number of args are less than 5
  {
    cerr << "Usage: " << argv[0] << " <IP address> <left_motor_port> <right_motor_port> <sensor_port> <hz>" << endl;
    cout <<"options for motor_port : outA outB outC outD"<<endl;
    cout<<"options for sensor_port : in1 in2 in3 in4"<<endl;

    return 1;
  }
  int milliseconds = 100;
  if(argc==6)
    milliseconds = 1000/atoi(argv[5]);
    string left_motor_port(argv[2]);
    string right_motor_port(argv[3]);
    string sensor_port(argv[4]);
   
  // TODO: Check if both are of same type
  srvrIP = (argv[1]);
  left_motor = motor(left_motor_port); 
  right_motor = motor(right_motor_port);
  s = sensor(sensor_port);
    if(s.connected())  // sensor object s will not be used hereafter
    cout<<"Gyroscope connected"<< endl;  

    else 
     {
         cerr << "Gyroscope not connected "<< endl;
         return 1;
     }   
    if(left_motor.connected()&&right_motor.connected()) 
    cout<<"Both motors connected "<<endl;
    else
        {
            cerr<<"Motors are not connected! Check and try again"<<endl;
            return 1;
        } 

//---------------------------------------------------------------------------------------------------------------------------------------------
  // TODO: Check if both were initialised

  left_motor.reset();
 // left_motor.set_position(0);
  left_motor.set_speed_regulation_enabled("on");

  right_motor.reset();
 // right_motor.set_position(0);
  right_motor.set_speed_regulation_enabled("on");

nh.initNode(srvrIP); // argv[1] is the ip address of the computer
nh.advertiseService(server);

// JUST TO KNOW IF THE NODE IS ALIVE

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg); // this publisher is just to show that the node is alive
char hello[13] = "hello world!";
nh.advertise(chatter);
str_msg.data = hello;

        while(1) {cout<<"service is being advertised. Waiting for request"<<endl;
               chatter.publish(&str_msg);
               nh.spinOnce();
               sleep(1);
        }
  
return 0;
}





