#ifndef UDP_SENDER_PORT_CLASS_FAULT_TEST_H
#define UDP_SENDER_PORT_CLASS_FAULT_TEST_H
/**
 * UDP_Sender_Port_class_fault_test.h
 */
#include <ros/ros.h>
#include <stdio.h> //printf
#include <string.h> //memset
#include <stdlib.h> //exit(0);
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "Buffer_Interface_class.h"
#include "Port_Superclass.h"

class UDP_Sender_Port_Fault_Test: public Buffer_Interface, public Port
{
public:
	//   Members 
	std::string ADDR_LOCAL, ADDR_OTHER;
	int PORT_LOCAL, PORT_OTHER;
    struct sockaddr_in si_other, si_local;
    int s, i;
    unsigned int slen=sizeof(si_other);
  
	//Methods
	UDP_Sender_Port_Fault_Test();        //constructor - opens port
	~UDP_Sender_Port_Fault_Test();       //destructor - closes port
	bool port_read();
	bool port_write();
};

#endif /* UDP_SENDER_PORT_CLASS_FAULT_TEST_H */
