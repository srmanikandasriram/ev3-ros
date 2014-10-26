/*
 * Robot controller using C++ binding of LEGO EV3
 *
 * Copyright (c) 2014 - S.R.Manikandasriram
 *
 * This program is distributed under The MIT License.
*/

#pragma once

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

class ev3client
{
protected:
  int init_socket();
  int read_socket(int,char*);

  int _socketfd, _clientsd;
  struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
  struct addrinfo *host_info_list; // Pointer to the linked list of host_info's.
  const char *_port;
  float x = 0.0, y = 0.0, t = 0.0;
  float R = 0.03, L = 0.12, d2r = M_PI/180.0;
  bool running = true;
  motor leftmotor = motor(OUTPUT_B), rightmotor = motor(OUTPUT_C), grippermotor = motor(OUTPUT_A);
  int encoder[2] = {0};

public:
  ev3client(unsigned int p = 1);
  ~ev3client()
  {
    close(_clientsd);
    freeaddrinfo(host_info_list);
    close(_socketfd);
  }
  void odometry();
  void controller();

  thread odom;
  thread drive;
};


