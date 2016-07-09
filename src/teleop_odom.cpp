/*Note : The ev3 listens to the topic turtlebot_teleop/cmd_vel which is published by the turtlebot_teleop_key.   It can be obtained by running the following command in the host,
   $ rosrun turtlebot_teleop turtlebot_teleop_key */

//Header files required to run ROS
#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

//Header files required to run EV3dev
#include <iostream>
#include "ev3dev.h"
#include <thread>
#include <string.h>
#include <math.h>
#include <chrono>
//#include <errno.h>

using namespace std;
using namespace ev3dev;//
int k = 0, q = 0;

ros::NodeHandle  nh;
char *rosSrvrIp ;//IP address of the master
const float deg2rad = M_PI/180.0;//conversion from degrees to radians
float vx, wt, vl, vr, L = 0.12,R = 0.03;
/*vx=velocity of centroid, wt=angular velocity of cenintroid, 
 vr,vl= velocity of wheels,L=distance between wheels R = radius of wheel*/

float x = 0.0, y = 0.0, t = 0.0, t_offset = 0.0;//setting up values of (x,y,t=theta),t_offset is for callibration
int encoder[2]={0,0}, prev_encoder[2] = {0,0}, dl, dr;

motor left_motor = OUTPUT_B, right_motor =  OUTPUT_C;//randomly initialised 
sensor gsense = sensor(INPUT_4) ;//randomly initialised 

float speed = R*deg2rad ;//conversion factor of linear speed to angular speed in rad/s

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
ros::Subscriber<geometry_msgs::Twist> cmd_sub("turtlebot_teleop/cmd_vel", vel_cb );
/* 
Telling the ROS server that we're subscribing from the said topic
The topic mentioned here is the one that publishes the velocaity commands through keyboard teleop.
Decalring the call back function here.
 */

void odometry() //function to calculate the odometry 
{
        int encoder[2]={0,0}, prev_encoder[2] = {0,0}, dl, dr;
        while(1)
        {
        encoder[0] = right_motor.position(); 
        encoder[1] = left_motor.position();

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

        //cout<<"theta = "<<t <<", q = "<<q<<endl;    
         
         //converting angle rotated by wheels to linear displacement and printing it 
        x += cos(t)*speed*(dl+dr)/2.0;
        y += sin(t)*speed*(dl+dr)/2.0;

        
        
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
        cerr << "Usage: " << argv[0] << " <IP address> <left_motor_port> <right_motor_port> <sensor_port> <hz>" << endl;
        cout <<"options for motor port: outA outB outC outD \n"<<"options for sensor port : in1 in2 in3 in4 \n";
        return 1;
    }
    int milliseconds = 100;//sets up frequency of publishing to the ROS server
    if(argc==6)
        milliseconds = 1000/atoi(argv[5]);

    string left_motor_port (argv[2]);
    string right_motor_port (argv[3]);
    string sensor_port (argv[4]);


    //Initialising motors and checking if connected 
    left_motor = motor(left_motor_port);
    right_motor = motor(right_motor_port);

    if(left_motor.connected()&&right_motor.connected())
        cout<<"both motors are connected"<<endl;
    else
        cout<<"error in connecting motors";
    if(gsense.connected())
        cout<<"gyro is connected"<<endl;
    else
        cout<<"error in connecting gyro";

    
    rosSrvrIp = (argv[1]);
    nh.initNode(rosSrvrIp);//setup proxy 
    nh.subscribe(cmd_sub);//subscribing to the topic to receive velocity commands
    
    t_offset = -gsense.value()*deg2rad;//obtaining initial angle to callibrate
    cout<<"t_offset = "<< t_offset <<endl;

    //configuring the motors
    left_motor.reset();
    left_motor.set_position(0);
    left_motor.set_speed_regulation_enabled("on");
    right_motor.reset();
    right_motor.set_position(0);
    right_motor.set_speed_regulation_enabled("on");

    // Set mode
    gsense = sensor(sensor_port);
    string mode="GYRO-G&A";
    gsense.set_mode(mode);
    
    //checking if gyroscope is connected 
    if(gsense.connected())
        cout<<gsense.type_name()<<"sensor is connected "<< endl;
        
    else 
        cout<<"gyro not connected";

    cout<<" Initialiased motors and gyroscope."<<endl;
    
    //time variables for transform broadcaster 
    ros::Time current_time, last_time;
    current_time = nh.now();
    last_time = nh.now();

     
    nav_msgs::Odometry odom_msg;//creating a msg of odometry type to publish odometry 
    ros::Publisher odom_pub("odom", &odom_msg);//odom is the topic and odom_msg is the message that ev3 publishes
    nh.advertise(odom_pub);

    tf::TransformBroadcaster odom_broadcaster;
    odom_broadcaster.init(nh);
    if (! nh.getParam("R", &R))
        { 
             R = 0.03;
            speed = R*deg2rad;
        }

    if (! nh.getParam("L", &L))
    { 
         L = 0.12;
    }
    
    cout<<" Retrieved params R,L : "<<R<<","<<L<<endl;

    thread odom (odometry);//'threading' odometry and the main functions
    while(1)
        {

        nh.spinOnce();               // check for incoming messages
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
