
/*the srv file headers_ev3/go_to_goal.h should be of the form  

float64 xg
float64 yg
---
float64 x
float64 y
float64 t

This program enables the robot to go to the co-ordinate (xg,yg) passed as request in the rosservice call
*/


//Header files required to run ROS
#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <headers/gtg.h>

//Header files required to run EV3dev
#include <iostream>
#include "ev3dev.h"
#include <thread>
#include <string.h>
#include <math.h>
#include <chrono>
//#include <errno.h>

using namespace std;
using namespace ev3dev;
using headers::gtg;

char* ROSSRVR_IP = "192.168.1.9" ;
ros::NodeHandle  nh;
//string ns = "/robot3/";
const float deg2rad = M_PI/180.0;//conversion from degrees to radians
float xc = 0, yc = 0 ,tc = 0; //current co-ordinates 
float x = 0.0, y = 0.0, t = 0.0, t_offset = 0.0;//dynamic value of co-ordinates 
float vx = 0, wt = 0, vl, vr,td = 0,t_error = 0,cerror = 0,L = 0.12,tIerror = 0,tDerror = 0, old_t_error = 0;//t_error = error in theta
float t_kp = 0 , t_ki = 0 , t_kd = 0;
char* srvrIP;
motor left_motor = OUTPUT_A, right_motor = OUTPUT_A;
sensor gsense = INPUT_4;
float R=0.03, speed = R*deg2rad ;//conversion factor of linear speed to angular speed in rad/s; 
int encoder[2]={0,0}, prev_encoder[2] = {0,0}, dl, dr,k=0,q=0 ;
string left_motor_port, right_motor_port, sensor_port ;


 
float t_PID()
{ 
  t_kp = 4; t_ki = 0; t_kd = 0;//setting PID gains
  float  omega = 0 ; 
  tDerror = t_error - old_t_error;
  tIerror = tIerror + t_error;
  omega = t_kp * t_error + t_ki * tIerror + t_kd * tDerror;
  old_t_error = t_error;
  //cout << "omega = "<< omega << endl;
  return omega ;
}

void svcCallback(const gtg::Request & req, gtg::Response & res)
{
    int count = 0;
    float xg, yg;
    xg = req.xg ; //obtaining goal's x-coordinate
    yg = req.yg ; //obtaining goal's y-coordinate
    td = atan((yg - yc)/(xg - xc));//calculating theta desired

    t_error = td - tc ;
    while((abs(xc-xg) >0.005)||(abs(yc-yg)>0.005))//setting tolerance as 0.005
    {
    
      count++;
      td = atan((yg - yc)/(xg - xc));//calculating theta desired
      if((xg-xc>0)&&(yg-yc>0) || (xg-xc>0)&&(yg-yc<0))
        td = td + 0;
      if((xg-xc<0)&&(yg-yc>0)) 
        td = M_PI - abs(td);
      if((xg-xc<0)&&(yg-yc<0))
        td = -M_PI + td;
      cout<<"desired = "<<td/deg2rad <<endl; 
      cout<< "current ="<<tc/deg2rad <<endl;
      t_error = td - tc ;
      wt = t_PID();

      /* bang bang control
      if(t_error > 0)
        wt = 0.2;
      else if(t_error < 0)
        wt = -0.2;*/
      
      vx = 0.1;
      vr = (vx+L/2*wt)/(speed); 
      vl = (vx-L/2*wt)/(speed);
      //cout<<"left , right = "<<vl <<" "<<vr <<endl;
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
      res.x = xc ; 
      res.y = yc ;
      res.t = tc/deg2rad ;
      td = 0;
      tIerror = 0;
      old_t_error = 0;
  cout <<"error = "<<t_error<< endl; 
  cout<<"counts = "<< count <<endl;
  cout<<"Service request message: ("<<req.xg<<","<<req.yg<<") received \n ";//responding with:("<< res.x<<","<<res.y<<","<<res.t/deg2rad<<")\n";
  cout<<"Current co-ordinates are \n (x,y,theta(degress)) == ("<<xc<<","<<yc<<","<<tc/deg2rad<<") \n";
  vl = 0.0 , vr = 0.0 ;
  left_motor.set_speed_sp(0);
  left_motor.set_command("run-forever"); 
  right_motor.set_speed_sp(0);
  right_motor.set_command("run-forever"); 
}
ros::ServiceServer<gtg::Request, gtg::Response> server("gtg_srv",&svcCallback);
void odometry() //function to calculate the odometry 
{
        int encoder[2]={0,0}, prev_encoder[2] = {0,0}, dl, dr;
        while(1)
        {
        encoder[0] = left_motor.position(); 
        encoder[1] = right_motor.position();

        //obtaining angle rotated by left and right wheels in 100 ms
        dl = encoder[0]-prev_encoder[0];
        dr = encoder[1]-prev_encoder[1];

        t = -gsense.value()*deg2rad - t_offset;//storing angle in radians after callibration  

        //making sure the angle lies in [-π,π]
        t = t/deg2rad;
        if(t>0)
        {
          k = t/360;
          t = t - k*360;
          if(t>0&&t<90)
            q =1; 
          if(t>90&&t<180)
            q = 2;
          if(t>180&&t<270)
            q = 3;
          if(t>270&&t<360)
            q = 4;
          if(q==1||q==2)
            t = t + 0;
          if(q==4||q==3)
            t = -360 + t ;
        }

        if(t<0)
        {
          k = -t/360;
          t = t + k*360;
          if(t<0 && t>-90)
            q =4; 
          if(t<-90&&t>-180)
            q = 3;
          if(t<-180&&t>-270)
            q = 2;
          if(t<-270&&t>-360)
            q = 1;
          if(q==4||q==3)
            t = t + 0;
          if(q==1||q==2)
            t = 360 + t ;
        }


        t*=deg2rad;
        tc = t;
        //cout << "current value = "<<tc<<endl;

        //converting angle rotated by wheels to linear displacement and printing it 
        x += cos(t)*speed*(dl+dr)/2.0; xc = x;
        y += sin(t)*speed*(dl+dr)/2.0; yc = y;
        //cout<<"(x,y,theta) = "<<"("<<x<<","<<y<<","<<t/deg2rad<<")"<<endl ;


        
        prev_encoder[0] = encoder[0];
        prev_encoder[1] = encoder[1];

        //motor.speed() returns speed in rad/s
        vl = left_motor.speed();
        vr = right_motor.speed();
        
        //remmapping linear and angular velocity of centroid from vl, vr
        vx = (vl+vr)/2*speed;
        wt = (vl-vr)/L*speed;

        this_thread::sleep_for(chrono::milliseconds(100));
        }
}
int main(int argc, char* argv[])
{
  if(argc<5)
  {
    cerr<<"Usage: "<<argv[0]<<" <IP address> <left motor port> <right motor port> <<gyro sensor port>\n";
    cout<<"motor port options : outA outB outC outD \n";
    cout<<"sensor port options : inA inB inC inD \n";
    return 1;
  }
  else 
  {
    
    ROSSRVR_IP = (argv[1]);
    left_motor_port = (argv[2]);
    right_motor_port = (argv[3]);
    sensor_port = (argv[4]);
  }

  left_motor = motor(left_motor_port);
  left_motor.reset();
  left_motor.set_position(0);
  left_motor.set_speed_regulation_enabled("on");

  right_motor = motor(right_motor_port);
  right_motor.reset();
  right_motor.set_position(0);
  right_motor.set_speed_regulation_enabled("on");


  nh.initNode(ROSSRVR_IP);
  nh.advertiseService(server);

  int milliseconds = 100;

  // Set mode
  gsense = sensor(sensor_port);
  string mode="GYRO-G&A";
  gsense.set_mode(mode);
  t_offset = -gsense.value()*deg2rad;//obtaining initial angle to callibrate
  cout<<"gyroscope is callibrated"<<endl ;
  thread odom (odometry);//'threading' odometry and the main functions

  while(1) 
  {
    nh.spinOnce();
    sleep(1);
    this_thread::sleep_for(std::chrono::milliseconds(milliseconds)); // sets frequency 
  }


    odom.join();
   
    return 0;
}
