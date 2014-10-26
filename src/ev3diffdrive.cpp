/*
 * Robot controller using C++ binding of LEGO EV3
 *
 * Copyright (c) 2014 - S.R.Manikandasriram
 *
 * This program is distributed under The MIT License.
*/

#include "ev3dev.h"

#include <iostream>
#include <fstream>
#include <thread>

#define _USE_MATH_DEFINES
#include <math.h>

#include <cstring> 
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

using namespace std;
using namespace ev3dev;

#define TCP_PORT "5555"

motor driveMotors[2] = {
	motor(OUTPUT_B),
	motor(OUTPUT_C)
};
motor gripperMotor = motor(OUTPUT_A);
sensor ultrasonic = sensor(INPUT_3);

float x = 0.0, y = 0.0, t = 0.0;
float R = 0.03, L = 0.12, d2r = M_PI/180.0;

bool running = true;

int socketfd ; // The socket descripter
struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
struct addrinfo *host_info_list; // Pointer to the linked list of host_info's.

int init_socket()
{

    int status;

    // The MAN page of getaddrinfo() states "All  the other fields in the structure pointed
    // to by hints must contain either 0 or a null pointer, as appropriate." When a struct
    // is created in c++, it will be given a block of memory. This memory is not nessesary
    // empty. Therefor we use the memset function to make sure all fields are NULL.
    memset(&host_info, 0, sizeof host_info);

    // cout << "Setting up the structs..."  << std::endl;

    host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
    host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.
    host_info.ai_flags = AI_PASSIVE;     // IP Wildcard

    // Now fill up the linked list of host_info structs with google's address information.
    status = getaddrinfo(NULL, TCP_PORT, &host_info, &host_info_list);
    // getaddrinfo returns 0 on succes, or some other value when an error occured.
    // (translated into human readable text by the gai_gai_strerror function).
    if (status != 0)  cout << "getaddrinfo error" << gai_strerror(status) ;


    // cout << "Creating a socket..."  << std::endl;
    socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype,
                      host_info_list->ai_protocol);
    if (socketfd == -1)  cout << "socket error " ;

    // cout << "Binding socket..."  << std::endl;
    // we use to make the setsockopt() function to make sure the port is not in use
    // by a previous execution of our code. (see man page for more information)
    int yes = 1;
    status = setsockopt(socketfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
    status = bind(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
    if (status == -1)  cout << "bind error" << std::endl ;

    // cout << "Listen()ing for connections..."  << std::endl;
    status =  listen(socketfd, 5);
    if (status == -1)  cout << "listen error" << std::endl ;

    int client_sd;
    struct sockaddr_storage their_addr;
    socklen_t addr_size = sizeof(their_addr);
    client_sd = accept(socketfd, (struct sockaddr *)&their_addr, &addr_size);
    if (client_sd == -1)
    {
        cout << "listen error" << std::endl ;
    }
    /*else
    {
         cout << "Connection accepted. Using new socketfd : "  <<  new_sd << std::endl;
    }*/

    return client_sd;
}

int read_socket(int client_sd, char* incomming_data_buffer)
{
    // cout << "Waiting to recieve data..."  << std::endl;
    ssize_t bytes_recieved;
    bytes_recieved = recv(client_sd, incomming_data_buffer,1024, 0);
    // If no data arrives, the program will just wait here until some data arrives.
    if (bytes_recieved == 0)  cout << "host shut down." << std::endl ;
    if (bytes_recieved == -1) cout << "recieve error!" << std::endl ;
    // cout << bytes_recieved << " bytes recieved :" << std::endl ;
    incomming_data_buffer[bytes_recieved] = '\0';
    // cout << incomming_data_buffer << std::endl;

    return bytes_recieved;
}

void write_socket(int client_sd, char* msg)
{
    // cout << "send()ing back a message..."  << std::endl;
    int len;
    ssize_t bytes_sent;
    len = strlen(msg);
    bytes_sent = send(client_sd, msg, len, 0);
}

void close_server(int client_sd)
{
	// cout << "Stopping server..." << std::endl;
    close(client_sd);
	freeaddrinfo(host_info_list);
	close(socketfd);
}

void odometry()
{
	int encoder[2] = {0}, prev_encoder[2] = {0};
	int dl = 0, dr = 0;

	cout<<"Beginning odometry..."<<endl;
  	while(running)
  	{
		encoder[0] = driveMotors[0].position();
		encoder[1] = driveMotors[1].position();
		dl = encoder[0]-prev_encoder[0];
		dr = encoder[1]-prev_encoder[1];
		x += cos(t)*d2r*R*(dl+dr)/2;
		y += sin(t)*d2r*R*(dl+dr)/2;
		t += (R/L)*(dl-dr)*d2r;
		prev_encoder[0] = encoder[0];
		prev_encoder[1] = encoder[1];
	}
	cout<<"Stopped odometry..."<<endl;
}

void controller(int client_sd)
{
	char command[1024], reply[1024];
	float vx, wt, vl, vr;
	float speed = R*d2r;
	cout<<"Beginning controller..."<<endl;
	while(strcmp(command,"q")!=0)
	{
		read_socket(client_sd, command);
    if(strcmp(command,"e")!=0){
			sscanf(command,"(%f,%f)",&vx,&wt);
			vl = (vx+L/2*wt)/speed;
			vr = (vx-L/2*wt)/speed;
			cout<<"command: "<<vl<<","<<vr<<endl;
			if(vl!=0)
			{
				driveMotors[0].set_pulses_per_second_setpoint(vl);
				driveMotors[0].run(true);
			}
			else
			{
				driveMotors[0].set_pulses_per_second_setpoint(0);
				driveMotors[0].run(false);	
			}
			if(vr!=0)
			{
				driveMotors[1].set_pulses_per_second_setpoint(vr);
				driveMotors[1].run(true);
			}
			else
			{
				driveMotors[1].set_pulses_per_second_setpoint(0);
				driveMotors[1].run(false);	
			}
		}
    sprintf(reply, "%f,%f,%f,%f",x,y,t,ultrasonic.value()/1000.0);
		write_socket(client_sd,reply);
	}
	write_socket(client_sd,"quit");
	close_server(socketfd);
	running = false;
	cout<<"Closing controller..."<<endl;
	return;
}

int main()
{
	// Initialize the motors
	driveMotors[0].reset();
	driveMotors[1].reset();
	driveMotors[0].set_position(0);
	driveMotors[1].set_position(0);

	driveMotors[0].set_run_mode("forever");
	driveMotors[1].set_run_mode("forever");
	driveMotors[0].set_stop_mode("brake");
	driveMotors[1].set_stop_mode("brake");
	driveMotors[0].set_regulation_mode("on");
	driveMotors[1].set_regulation_mode("on");
		
	//auto start = chrono::high_resolution_clock::now();
	//auto end = chrono::high_resolution_clock::now();
	//cout << "time was " << chrono::duration_cast<chrono::nanoseconds>(end - start).count()/N<<endl;
	
	int client_sd = init_socket();
	thread odom (odometry);
	thread drive (controller, client_sd);

	drive.join();
	odom.join();

	cout<<"Final pose: "<<x<<","<<y<<","<<t/d2r<<endl;
	return 0;
}