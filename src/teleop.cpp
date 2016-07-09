/*Note : The ev3 listens to the topic turtlebot_teleop/cmd_vel which is published by the turtlebot_teleop_key.   It can be obtained by running the following command in the host,
   $ rosrun turtlebot_teleop turtlebot_teleop_key */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <iostream>
#include "ev3dev.h"
#include <thread>
#include <string.h>
#include <errno.h>

using namespace std;
using namespace ev3dev;
using namespace std;
using namespace ev3dev;
ros::NodeHandle  nh;
char *rosSrvrIp = "192.168.1.9";//randomly initialised, proper IP to be entered as arguements for main function 
const float deg2rad = M_PI/180.0;


motor left_motor = OUTPUT_C, right_motor = OUTPUT_B;//randomly initialised 
string left_motor_port,right_motor_port ; 


float speed = 0.028*deg2rad ; //0.028 is the radius of the wheel 

 void vel_cb(const geometry_msgs::Twist& cmd)
{
        float vx, wt, vl, vr, L = 0.12;
    vx = cmd.linear.x;
    wt = cmd.angular.z;

    //mapping given linear and angular velocities to motor velocities
    vr = (vx+L/2*wt)/(speed); 
    vl = (vx-L/2*wt)/(speed);
    cout << "received " << vx << "," << wt << "=>" << vl <<"," << vr << endl;
    left_motor.set_speed_regulation_enabled("on");
    right_motor.set_speed_regulation_enabled("on");
    if(vl!=0)
    {
        left_motor.set_speed_sp(vl);
        left_motor.set_command("run-forever");
    }
    else
    {
        left_motor.set_speed_sp(0);
        left_motor.set_command("run-forever");  
    }
    if(vr!=0)
    {
        right_motor.set_speed_sp(vr); 
        right_motor.set_command("run-forever");
    }
    else
    {
        right_motor.set_speed_sp(0);
        right_motor.set_command("run-forever"); 
    }
   
}
ros::Subscriber<geometry_msgs::Twist> cmd_sub("turtlebot_teleop/cmd_vel", vel_cb );

int main(int argc, char* argv[])
{
       if(argc != 4 )
       {
        cout<<"Usage : "<<argv[0]<<" <IP address> <left motor port> <right motor port>"<<endl;
        cout<<"Motor port options: outA outB outC outD"<<endl;
        return 1;
       }
       if(argc == 4)
        {
        right_motor_port = (argv[3]);
        left_motor_port = (argv[2]);
        left_motor = motor(left_motor_port);
        right_motor = motor(right_motor_port);
        }

        if(left_motor.connected()&&right_motor.connected())
            cout<<"motors are connected \n";
        else 
            cout<<"motors are not connected \n";

        rosSrvrIp = (argv[1]);
        nh.initNode(rosSrvrIp);
        nh.subscribe(cmd_sub);

        while(1) 
        {
                  sleep(1);
                  nh.spinOnce();//checks for incoming message 
        }
        return 0;
}
