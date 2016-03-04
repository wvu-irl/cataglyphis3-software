#ifndef UDP_RECEIVER_PORT_CLASS_H
#define UDP_RECEIVER_PORT_CLASS_H
/**
 * UDP_Receiver_Port_class.h
 */
#include <ros/ros.h>
#include <stdio.h>  //printf
#include <stdlib.h> //exit
#include <string.h> //memset
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

//#include <hsm/HSM_Detector.h>

#include "Buffer_Interface_class.h"
#include "Port_Superclass.h"

class UDP_Receiver_Port: public Buffer_Interface, public Port
{
public:
	//   Members 
	std::string ADDR_LOCAL, ADDR_OTHER;
	int PORT_LOCAL, PORT_OTHER;
	struct sockaddr_in si_local, si_incoming;
    int s, i;
    unsigned int slen = sizeof(si_incoming);
    int recv_len;
//	HSM_Detector det;
    struct timeval tv;


	//Methods
	UDP_Receiver_Port();        //constructor - opens port
	~UDP_Receiver_Port();       //destructor - closes port
	bool port_read();
	bool port_write();
};

#endif /* UDP_RECEIVER_PORT_CLASS_H */
