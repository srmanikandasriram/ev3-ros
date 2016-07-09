
/*the srv file headers_ev3/go_to_goal.h should be of the form  

float64 r
float64 al
---
float64 x
float64 y
float64 t



This program enables the robot to trace an arc with the mentioned radius r and arc length al(degrees) passed as request in rosservice call 
*/


//Header files required to run ROS
#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <headers/arc.h>

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
using headers::arc;

char* ROSSRVR_IP = "192.168.1.11" ;
ros::NodeHandle  nh;
//string ns = "/robot3/";
const float deg2rad = M_PI/180.0;//conversion from degrees to radians
float xc = 0, yc = 0 ,tc_arc = 0; //current co-ordinates 
float x = 0.0, y = 0.0, t = 0.0, tarc_offset = 0.0;//dynamic value of co-ordinates 
float vx = 0, wt = 0, vl, vr,td = 0,t_error = 0,cerror = 0,tIerror = 0,tDerror = 0, old_t_error = 0, tcovered = 0;//t_error = error in theta
float t_kp = 0 , t_ki = 0 , t_kd = 0 , rt_error = 0, rtd = 0 , t_arc = 0;
char* srvrIP;
motor left_motor = OUTPUT_B, right_motor = OUTPUT_C;
string left_motor_port, right_motor_port, sensor_port ;
sensor gsense = INPUT_4;
float R=0.03,L = 0.12;//Radius of the whheel and the distance between the two wheels 

float speed = R*deg2rad ;//conversion factor of linear speed to angular speed in rad/s; 
int encoder[2]={0,0}, prev_encoder[2] = {0,0}, dl, dr,k=0,q=0, r = 1 ;



 
float t_PID()
{ 
  t_kp = 6; t_ki = 0; t_kd = 0;//setting PID gains
  float  omega = 0 ; 
  tDerror = t_error - old_t_error;
  tIerror = tIerror + t_error;
  omega = t_kp * t_error + t_ki * tIerror + t_kd * tDerror;
  old_t_error = t_error;
  omega =  omega +  0.1*rt_error; 

  //cout << "omega = "<< omega << endl;
  return omega ;
}

void svcCallback(const arc::Request & req, arc::Response & res)
{
    int count = 0;
    float  rad, arct, xcentre, ycentre ;
    rad = req.rg ; //obtaining radius
    arct = req.arcl*deg2rad ; //obtaining arc length in terms of angle in degrees  
    xcentre = xc;
    ycentre = yc + rad;
    
    t_error = td - tc_arc ;

    
    while(abs(abs(tcovered/deg2rad)- abs(arct/deg2rad)) > 1)
    {
      
      

      td = atan((yc - ycentre)/(xc - xcentre));//calculating theta desired

      if((xc-xcentre>0)&&(yc-ycentre>0))//1st quadrant
        {
          td = M_PI/2 + abs(td) ;
          tcovered = M_PI/2 + abs(atan((yc-ycentre)/(xc-xcentre)))  ;
          //cout << "1111 "<<abs(atan((yc-ycentre)/(xc-xcentre)))/deg2rad<<endl;
          if(sqrt((xc-xcentre)*(xc-xcentre)+(yc-ycentre)*(yc-ycentre)) > rad + 0.0001 )
            rtd = M_PI + abs(atan((yc-ycentre)/(xc-xcentre))) ;

          if(sqrt((xc-xcentre)*(xc-xcentre)+(yc-ycentre)*(yc-ycentre)) < rad - 0.0001 )  
            rtd = abs(atan((yc-ycentre)/(xc-xcentre))) ;      
         }

      if((xc-xcentre<0)&&(yc-ycentre>0))//2nd quadrant
        {
          td = 3*M_PI/2 - abs(td);
          tcovered = 3*M_PI/2 - abs(atan((yc-ycentre)/(xc-xcentre))) ;
          //cout << "22222 "<<abs(atan((yc-ycentre)/(xc-xcentre)))/deg2rad<<endl;
          if(sqrt((xc-xcentre)*(xc-xcentre)+(yc-ycentre)*(yc-ycentre)) > rad + 0.0001 )
            rtd = 2*M_PI - abs(atan((yc-ycentre)/(xc-xcentre))) ; 

          if(sqrt((xc-xcentre)*(xc-xcentre)+(yc-ycentre)*(yc-ycentre)) < rad - 0.0001 )  
            rtd = M_PI - abs(atan((yc-ycentre)/(xc-xcentre)));
        }

      if((xc-xcentre < 0)&&(yc-ycentre<0))//3rd quadrant
        {
          td = 3*M_PI/2 + abs(td) ;
          tcovered = 3*M_PI/2 + abs(atan((yc-ycentre)/(xc-xcentre)));
          //cout << "333333 "<<abs(atan((yc-ycentre)/(xc-xcentre)))/deg2rad<<endl;
          if(sqrt((xc-xcentre)*(xc-xcentre)+(yc-ycentre)*(yc-ycentre)) > rad + 0.0001 )
            rtd = abs(atan((yc-ycentre)/(xc-xcentre)));

          if(sqrt((xc-xcentre)*(xc-xcentre)+(yc-ycentre)*(yc-ycentre)) < rad - 0.0001 )  
            rtd = M_PI + abs(atan((yc-ycentre)/(xc-xcentre)));
        }  
      
      if((xc-xcentre>0)&&(yc-ycentre<0))//4th quadrant
        {
          td = M_PI/2 - abs(td) ;
          tcovered = M_PI/2 - abs(atan((yc-ycentre)/(xc-xcentre))) ;
          //cout << "44444 "<<abs(atan((yc-ycentre)/(xc-xcentre)))/deg2rad<<endl;
          if(sqrt((xc-xcentre)*(xc-xcentre)+(yc-ycentre)*(yc-ycentre)) > rad + 0.0001 )
            rtd = M_PI - abs(atan((yc-ycentre)/(xc-xcentre))) ; 

          if(sqrt((xc-xcentre)*(xc-xcentre)+(yc-ycentre)*(yc-ycentre)) < rad - 0.0001 )  
            rtd = 2*M_PI -abs(atan((yc-ycentre)/(xc-xcentre)));
        }

        rt_error = rtd - tc_arc ; 
        res.tc = tcovered/deg2rad ;
        
      cout<<"desired = "<<arct/deg2rad<<endl; 
      cout<< "current ="<<tcovered/deg2rad<<endl;//(atan(xc/(yc-rad))/deg2rad) <<endl;
      t_error = td - tc_arc ;
      /*if(abs(t_error/deg2rad) > 360)
        {
          t_error = abs(td) - abs(tc);
          cout<<endl<<endl<<endl<<endl<<endl;
        }*/
      wt = t_PID();
      if(t_error/deg2rad > 350 || t_error/deg2rad > 350 )
        break ;
     
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
      
      td = 0;
      tIerror = 0;
      old_t_error = 0;

  cout <<"error = "<<t_error<< endl; 
  cout<<"Service request message: ("<<req.rg<<","<<req.arcl<<") received \n ";
  cout<<"Current co-ordinates are \n (x,y,theta(degress)) == ("<<xc<<","<<yc<<","<<tc_arc/deg2rad<<") \n";
  vl = 0.0 , vr = 0.0 ;
  left_motor.set_speed_sp(0);
  left_motor.set_command("run-forever"); 
  right_motor.set_speed_sp(0);
  right_motor.set_command("run-forever"); 
  tcovered = 0;
  tarc_offset = tarc_offset + tc_arc;
 /* t = 0;
  x = 0; y = 0;
  left_motor.reset();
  right_motor.reset();
  cout<<"......."<<x<<","<<y<<","<<t;*/

}
ros::ServiceServer<arc::Request, arc::Response> server("arc_srv",&svcCallback);
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

        t = -gsense.value()*deg2rad - tarc_offset;//storing angle in radians after callibration  
        //making sure the angle lies in [0,2π]
        t_arc = t;
        t_arc = t_arc/deg2rad;
        if(t_arc>0)
        {
          k = t_arc/360;
          t_arc = t_arc - k*360;

        }

        if(t_arc<0)
        {
          k = -t_arc/360;
          t_arc = t_arc + k*360;

        }


        t_arc*=deg2rad;
        tc_arc = t_arc;
        //cout << "current value = "<<tc<<endl;

        //converting angle rotated by wheels to linear displacement and printing it 
        x += cos(t)*speed*(dl+dr)/2.0; xc = x;
        y += sin(t)*speed*(dl+dr)/2.0; yc = y;
        cout<<"(x,y,theta) = "<<"("<<x<<","<<y<<","<<t/deg2rad<<")"<<endl ;


        
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
  gsense = sensor(sensor_port);

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
    ros::Time current_time, last_time;
  current_time = nh.now();
  last_time = nh.now();
  tf::TransformBroadcaster odom_broadcaster;
  odom_broadcaster.init(nh);
  nav_msgs::Odometry odom_msg;//creating a msg of odometry type to publish odometry 
  ros::Publisher odom_pub("odom", &odom_msg);//odom is the topic and odom_msg is the message that ev3 publishes
  nh.advertise(odom_pub);

  // Set mode
  //gsense = sensor(sensor_port);
  string mode="GYRO-G&A";
  gsense.set_mode(mode);
  tarc_offset = -gsense.value()*deg2rad;//obtaining initial angle to callibrate
  //cout<<"t_offset = "<< t_offset <<endl;
  cout<<"gyroscope is callibrated"<<endl ;
  thread odom (odometry);//'threading' odometry and the main functions

  while(1) 
  {
    nh.spinOnce();
    sleep(1);
    current_time = nh.now();

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(t);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        const char base_link_name[18] = "base_link";
        odom_trans.child_frame_id = base_link_name;
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);
        // broadcaster.sendTransform( tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, t, 1), tf::Vector3(x, y, 0)),ros::Time::now(),"odom", "base_link"));

        //next, we'll publish the odometry message over ROS
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";

        //set the position
        odom_msg.pose.pose.position.x = x; 
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        //set the velocity
        odom_msg.child_frame_id = base_link_name;
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = 0;
        odom_msg.twist.twist.angular.z = wt;

        //publish the message
        odom_pub.publish(&odom_msg);

        last_time = current_time;

    this_thread::sleep_for(std::chrono::milliseconds(milliseconds)); // sets frequency 
  }


    odom.join();
   
    return 0;
}
