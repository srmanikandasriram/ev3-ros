/*
 * Robot controller using C++ binding of LEGO EV3
 *
 * Copyright (c) 2014 - S.R.Manikandasriram
 *
 * This program is distributed under The MIT License.
*/

ev3client::ev3client(unsigned int p){
	leftmotor.reset();
	rightmotor.reset();
	leftmotor.set_position(0);
	rightmotor.set_position(0);

	leftmotor.set_run_mode("forever");
	rightmotor.set_run_mode("forever");
	leftmotor.set_stop_mode("brake");
	rightmotor.set_stop_mode("brake");
	leftmotor.set_regulation_mode("on");
	rightmotor.set_regulation_mode("on");

	_clientsd = init_socket();
  	odom = std::thread(odometry);
  	drive = std::thread(controller);
}

int ev3client::init_socket()
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
    status = getaddrinfo(NULL, "5555", &host_info, &host_info_list);
    // getaddrinfo returns 0 on succes, or some other value when an error occured.
    // (translated into human readable text by the gai_gai_strerror function).
    if (status != 0)  cout << "getaddrinfo error" << gai_strerror(status) ;


    // cout << "Creating a socket..."  << std::endl;
    _socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype,
                      host_info_list->ai_protocol);
    if (_socketfd == -1)  cout << "socket error " ;

    // cout << "Binding socket..."  << std::endl;
    // we use to make the setsockopt() function to make sure the port is not in use
    // by a previous execution of our code. (see man page for more information)
    int yes = 1;
    status = setsockopt(_socketfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
    status = bind(_socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
    if (status == -1)  cout << "bind error" << std::endl ;

    // cout << "Listen()ing for connections..."  << std::endl;
    status =  listen(_socketfd, 5);
    if (status == -1)  cout << "listen error" << std::endl ;

    int client_sd;
    struct sockaddr_storage their_addr;
    socklen_t addr_size = sizeof(their_addr);
    client_sd = accept(_socketfd, (struct sockaddr *)&their_addr, &addr_size);
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

int ev3client::read_socket(char* incomming_data_buffer)
{
    // cout << "Waiting to recieve data..."  << std::endl;
    ssize_t bytes_recieved;
    bytes_recieved = recv(_clientsd, incomming_data_buffer,1024, 0);
    // If no data arrives, the program will just wait here until some data arrives.
    if (bytes_recieved == 0)  cout << "host shut down." << std::endl ;
    if (bytes_recieved == -1) cout << "recieve error!" << std::endl ;
    // cout << bytes_recieved << " bytes recieved :" << std::endl ;
    incomming_data_buffer[bytes_recieved] = '\0';
    // cout << incomming_data_buffer << std::endl;

    return bytes_recieved;
}

void ev3client::write_socket(char* msg)
{
    // cout << "send()ing back a message..."  << std::endl;
    int len;
    ssize_t bytes_sent;
    len = strlen(msg);
    bytes_sent = send(_clientsd, msg, len, 0);
}

void ev3client::odometry()
{
	int prev_encoder[2] = {0};
	int dl = 0, dr = 0;
  	while(running)
  	{
		encoder[0] = leftmotor.position();
		encoder[1] = rightmotor.position();
		dl = encoder[0]-prev_encoder[0];
		dr = encoder[1]-prev_encoder[1];
		x += cos(t)*d2r*R*(dl+dr)/2;
		y += sin(t)*d2r*R*(dl+dr)/2;
		t += (R/L)*(dl-dr)*d2r;
		prev_encoder[0] = encoder[0];
		prev_encoder[1] = encoder[1];
	}
}

void ev3client::controller()
{
	char command[1024], reply[1024];
	float vx, wt, vl, vr;
	float speed = R*d2r;
	while(strcmp(command,"q")!=0)
	{
		read_socket(_clientsd, command);
		if(strcmp(command,"e")!=0){
			sscanf(command,"(%f,%f)",&vx,&wt);
			vl = (vx+L/2*wt)/speed;
			vr = (vx-L/2*wt)/speed;
			if(vl!=0)
			{
				leftmotor.set_pulses_per_second_setpoint(vl);
				leftmotor.run(true);
			}
			else
			{
				leftmotor.set_pulses_per_second_setpoint(0);
				leftmotor.run(false);	
			}
			if(vr!=0)
			{
				rightmotor.set_pulses_per_second_setpoint(vr);
				rightmotor.run(true);
			}
			else
			{
				rightmotor.set_pulses_per_second_setpoint(0);
				rightmotor.run(false);	
			}
		}
		sprintf(reply, "%f,%f,%f",x,y,t);
		write_socket(_clientsd,reply);
	}
	write_socket(_clientsd,"quit");
	running = false;
	odom.join();
	return;
}
