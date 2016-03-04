/**
 * UDP_Sender_Port_class.cpp
 * Implements UDP port_open (via constructor), port_close (via destructor), port_read (erroneous call), and port_write methods 
 */
#include "UDP_Sender_Port_class.h"

#include <stdint.h>
#include <stdio.h>

UDP_Sender_Port::UDP_Sender_Port()
{
	/**
	 * Open UDP port/socket
	 */
	//read launch parameters
	if(ros::param::get("ADDR_LOCAL", ADDR_LOCAL)==false) ADDR_LOCAL = "10.20.15.1";
	if(ros::param::get("ADDR_OTHER", ADDR_OTHER)==false) ADDR_OTHER = "10.20.15.2";
	if(ros::param::get("PORT_LOCAL", PORT_LOCAL)==false) PORT_LOCAL = 9000;
	if(ros::param::get("PORT_OTHER", PORT_OTHER)==false) PORT_OTHER = 9001;
	//create a UDP socket
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        printf("could not create socket\n");
    }
    
    // Set local address and port
    memset((char *) &si_local, 0, sizeof(si_local));
    si_local.sin_family = AF_INET;
    si_local.sin_port = htons(PORT_LOCAL);
    if (inet_aton(ADDR_LOCAL.c_str() , &si_local.sin_addr) == 0)
    {
        printf("inet_aton() local failed\n");
    }
    //bind socket to port
    if( bind(s, (struct sockaddr*)&si_local, sizeof(si_local) ) == -1)
    {
        printf("bind failed\n");
    }
 
 	// Set other address and port
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT_OTHER);
    if (inet_aton(ADDR_OTHER.c_str() , &si_other.sin_addr) == 0)
    {
        printf("inet_aton() other failed\n");
    }

}  

bool UDP_Sender_Port::port_read()
{
	printf("ERROR: call made to UDP_Sender::port_read()");

	return false;
}


bool UDP_Sender_Port::port_write()
{
	bool return_val = true;
	/**
	 * Write data to the port - 
	 */
	 //hexprint
	printf(" - - - - - - - write_buffer: "); 
	for(int i=0;i<wbuf_size;i++)
	{
		printf("%2.2hhX|",write_buffer[i]);
	}
	printf("\n"); 
	//--hexprint
	printf("before sendto()\n");
	//send the message
	if (sendto(s, write_buffer, wbuf_size , 0 , (struct sockaddr *) &si_other, slen)==-1)
	{
		printf("sendto failed\n");
		return_val = false;
	}
	return return_val;
}

UDP_Sender_Port::~UDP_Sender_Port()
{
	//
	// Close UDP port 
	//
    close(s);
}
