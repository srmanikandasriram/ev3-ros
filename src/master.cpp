//Header files required to run ROS
#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <headers/gtg.h>
#include <headers/orientation.h>
#include <headers/arc.h>
#include <headers/straight.h>

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
using headers::orientation;
using headers::arc;
using headers::straight;

char* ROSSRVR_IP = "192.168.1.9" ;
ros::NodeHandle  nh;
//string ns = "/robot3/";
const float deg2rad = M_PI/180.0;//conversion from degrees to radians
float xc = 0, yc = 0 ,tc = 0, tc_arc = 0; //current co-ordinates 
float x = 0.0, y = 0.0, t = 0.0, t_offset = 0.0 , tarc_offset = 0;//dynamic value of co-ordinates 
float vx = 0, wt = 0, vl, vr,td = 0,t_error = 0,cerror = 0,L = 0.12,tIerror = 0,tDerror = 0, old_t_error = 0;//t_error = error in theta
float t_kp = 0 , t_ki = 0 , t_kd = 0 ,rt_error = 0, rtd = 0 , t_arc = 0 , tcovered = 0 ,ti = 0 ;
char* srvrIP;
motor left_motor = OUTPUT_B, right_motor = OUTPUT_C;
sensor gsense = INPUT_4;
sensor us = INPUT_1;
float R=0.03, speed = R*deg2rad ;//conversion factor of linear speed to angular speed in rad/s; 
int encoder[2]={0,0}, prev_encoder[2] = {0,0}, dl, dr,k=0,q=0 ;
string left_motor_port, right_motor_port, sensor_port , usensor_port;


 
float t_PID()
{ 
  t_kp = 4; t_ki = 0; t_kd = 0;//setting PID gains
  float  omega = 0 ; 
  tDerror = t_error - old_t_error;
  tIerror = tIerror + t_error;
  omega = t_kp * t_error + t_ki * tIerror + t_kd * tDerror;
  old_t_error = t_error;
  omega =  omega +  0.1*rt_error;
  //cout << "omega = "<< omega << endl;
  return omega ;
}
void ori_svcCallback(const orientation::Request & req, orientation::Response & res)
{
    int count = 0;
    float xg, yg;
    xg = req.vecx ; //obtaining goal's x-coordinate
    yg = req.vecy ; //obtaining goal's y-coordinate
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
    while(abs(t_error) >=0.01)
    {
      count++;
      t_error = td - tc ;
      wt = t_PID();
      /* bang bang control
      if(t_error > 0)
        wt = 0.2;
      else if(t_error < 0)
        wt = -0.2;*/
      vx = 0;
      
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
      res.tf = tc ;
      td = 0;
      tIerror = 0;
      old_t_error = 0;
  cout <<"error = "<<t_error<< endl; 
  cout<<"counts = "<< count <<endl;
  cout<<"Service request message: ("<<req.vecx<<","<<req.vecy<<") received \n ";//responding with:("<< res.x<<","<<res.y<<","<<res.tf/deg2rad<<")\n";
  cout<<"Current co-ordinates are \n (x,y,theta(degress)) == ("<<xc<<","<<yc<<","<<tc/deg2rad<<") \n";
  vl = 0.0 , vr = 0.0 ;
  left_motor.set_speed_sp(0);
  left_motor.set_command("run-forever"); 
  right_motor.set_speed_sp(0);
  right_motor.set_command("run-forever"); 
}

void gtg_svcCallback(const gtg::Request & req, gtg::Response & res)
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

 void vel_cb(const geometry_msgs::Twist& cmd)//call back function for subscribing velocity commands from teleop
{
    
    vx = cmd.linear.x;//obtaining centroid velocity
    wt = cmd.angular.z;//obtaining angular velocity
    vr = (vx+L/2*wt)/(speed); //mapping velocity commands-
    vl = (vx-L/2*wt)/(speed); //to wheels
    
    //cout << "received " << vx << "," << wt << "=>" << vl <<"," << vr << endl;
    //uncomment above to see values of vl and vr after every conversion     
   
    if(vl!=0)
    {   
        
        left_motor.set_speed_sp(vl);//setting up speed for the left motor 
        left_motor.set_command("run-forever");
        cout<<vl;
    }
    else
    {
        left_motor.set_speed_sp(0);
        left_motor.set_command("run-forever");  
    }
    if(vr!=0)
    {   
        
        right_motor.set_speed_sp(vr); //setting up speed for the right motor 
        right_motor.set_command("run-forever");
        cout<<vr;
    }
    else
    {
        right_motor.set_speed_sp(0);
        right_motor.set_command("run-forever"); 
    }
   cout<<"(x,y,theta) = "<<"("<<x<<","<<y<<","<<t<<")"<<endl ;
}

void arc_svcCallback(const arc::Request & req, arc::Response & res)
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
  rt_error = 0;
 /* t = 0;
  x = 0; y = 0;
  left_motor.reset();
  right_motor.reset();
  cout<<"......."<<x<<","<<y<<","<<t;*/

}

void str_svcCallback(const headers::straight::Request &req , headers::straight::Response &res){ // The callback function
  //float vx, wt, vl, vr; // LOCAL VARIABLES DEFINED AGAIn
  speed = R*deg2rad;
  left_motor.set_speed_regulation_enabled("on");
  right_motor.set_speed_regulation_enabled("on");
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
    cout<<res.vr<<"..."<<ti<<endl;
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

ros::ServiceServer<headers::straight::Request, headers::straight::Response> str_server("straight_srv",&str_svcCallback); //SERVICE DEFINITION 
ros::ServiceServer<arc::Request, arc::Response> arc_server("arc_srv",&arc_svcCallback);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("turtlebot_teleop/cmd_vel", vel_cb );
ros::ServiceServer<gtg::Request, gtg::Response> gtg_server("gtg_srv",&gtg_svcCallback);
ros::ServiceServer<orientation::Request, orientation::Response> ori_server("ori_srv",&ori_svcCallback);





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
    cerr<<"Usage: "<<argv[0]<<" <IP address> <left motor port> <right motor port> <gyro sensor port> <ultrasonic sensor port> \n";
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
    usensor_port = (argv[5]);
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
  nh.advertiseService(arc_server);//trace arc services
  nh.advertiseService(ori_server);//orientation service
  nh.advertiseService(gtg_server);//go to goal service
  nh.subscribe(cmd_sub);//teleop messages
  nh.advertiseService(str_server);//go straight services

  int milliseconds = 100;

  // Set mode
  gsense = sensor(sensor_port);
  us = sensor(usensor_port);
  string mode="GYRO-G&A", umode = "US-DIST-CM";
  gsense.set_mode(mode);
  us.set_mode(umode);

  
        sensor_msgs::LaserScan us_msg;
        ros::Publisher us_pub("scan", &us_msg);
        nh.advertise(us_pub);
        int number = 10;
        float ranges[10]={0.0}, intensities[10] = {0};
        us_msg.header.frame_id = "laser_frame";
        us_msg.angle_min = -0.08;
        us_msg.angle_max = 0.08;
        us_msg.angle_increment = 0.16/number;
        us_msg.time_increment = 0.0001;
        us_msg.scan_time = 0.0001;
        us_msg.range_min = 0.01;
        us_msg.range_max = 2.55;
        us_msg.ranges_length = number;
        us_msg.intensities_length = number;

        cout<<" Advertised message"<<endl;

        ros::Time us_current_time, us_last_time;
        us_current_time = nh.now();
        us_last_time = nh.now();




  t_offset = -gsense.value()*deg2rad;//obtaining initial angle to callibrate
  cout<<"gyroscope is callibrated"<<endl ;
  
  ros::Time current_time, last_time;
  current_time = nh.now();
  last_time = nh.now();
  tf::TransformBroadcaster odom_broadcaster;
  odom_broadcaster.init(nh);
  nav_msgs::Odometry odom_msg;//creating a msg of odometry type to publish odometry 
  ros::Publisher odom_pub("odom", &odom_msg);//odom is the topic and odom_msg is the message that ev3 publishes
  nh.advertise(odom_pub);

  thread odom (odometry);//'threading' odometry and the main functions
  cout<<" Initialiased node."<<endl;
  
  if (! nh.getParam("R", &R)){ 
       R = 0.03;
       speed = R*deg2rad;
  }

  if (! nh.getParam("L", &L)){ 
       L = 0.12;
  }
  
  cout<<" Retrieved params R,L : "<<R<<","<<L<<endl;

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
     
                     us_current_time = nh.now();

                us_msg.header.stamp = us_current_time;
                float distance = us.value()/1000.0;
                //cout<< us.value()/1000.0 <<endl ;

            for (int i = 0; i < number; ++i)
            {
                ranges[i] = distance;
                intensities[i] = 255;
            }
            us_msg.ranges = ranges;
            us_msg.intensities = intensities;
                // cout<<" constructed message of size "<<sizeof(us_msg)<<endl;
            // for (int i = 0; i < number; ++i)
            // {
            //  cout<<us_msg.ranges[i]<<" ";
            // }
            us_pub.publish(&us_msg);
                //cout<<" Published message"<<endl;
                us_last_time = us_current_time;


        this_thread::sleep_for(std::chrono::milliseconds(milliseconds)); // sets frequency 
        
    
  }


    odom.join();
   
    return 0;
}
