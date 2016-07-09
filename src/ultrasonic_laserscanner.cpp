/*
*This program, takes data from the Lego's ultrasonic sensor and publishes it to the topic "scan" as Laserscan messages
*/
#include <ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>
#include "ev3dev.h"
#include <chrono>

char* srvrip;


using namespace std;
using namespace ev3dev;

ros::NodeHandle  nh;

float deg2rad = M_PI/180.0;

sensor us = INPUT_1;

int main(int argc, char* argv[])
{
        if(argc<3)
        {
                cerr << "Usage: " << argv[0] << " <socket> <sensor_port>" << endl;
                return 1;
        }
    srvrip = (argv[1]);
    nh.initNode(srvrip);

   
    string sensor_port (argv[2]);
    us = sensor(sensor_port);

    if(!us.connected())
        {
                cerr << "Sensor is not connected " << endl;
                return 1;
        }    

    else 
        cout<<"sensor connected"<<endl;   

        // Set mode
        string mode="US-DIST-CM";
        us.set_mode(mode);

        cout<<" Set mode"<<endl;

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

        ros::Time current_time, last_time;
        current_time = nh.now();
        last_time = nh.now();

    while(!nh.connected()) {nh.spinOnce();}

        // TODO: Test for frequency compliance and implementation of ros::Rate
        while(1){

                nh.spinOnce();               // check for incoming messages
                current_time = nh.now();

                us_msg.header.stamp = current_time;
                float distance = us.value()/1000.0;
                cout<< us.value()/1000.0 <<endl ;

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
                cout<<" Published message"<<endl;
                last_time = current_time;
                sleep(1.0);
                // cout<<" Next loop"<<endl;
        }
    return 0;

}