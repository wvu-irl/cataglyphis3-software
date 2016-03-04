/**
 * UDP_Receiver_Port_class.cpp
 * Implements UDP port_open (via constructor), port_close (via destructor), port_read, and port_write (erroneous call) methods 
 */
#include "UDP_Receiver_Port_class.h"

#include <stdint.h>
#include <stdio.h>

UDP_Receiver_Port::UDP_Receiver_Port()
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
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        printf("could not create socket\n");
    }
    
    //set timeout for recvfrom
	tv.tv_sec = 1;  /* 1 Sec Timeout */
	tv.tv_usec = 0;  // Not initializing this can cause strange errors
	setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
     
    // zero out the structure
    memset((char *) &si_local, 0, sizeof(si_local));
     
    si_local.sin_family = AF_INET;
    si_local.sin_port = htons(PORT_LOCAL);
    si_local.sin_addr.s_addr = htonl(INADDR_ANY);
     
    //bind socket to port
    if( bind(s, (struct sockaddr*)&si_local, sizeof(si_local) ) == -1)
    {
        printf("bind failed\n");
    }

}  

bool UDP_Receiver_Port::port_read()
{
	bool return_val = true;
	printf("Waiting for data...\n");
	fflush(stdout);
         
	//try to receive some data, this is a blocking call
	if ((recv_len = recvfrom(s, read_buffer, rbuf_size, 0, (struct sockaddr *) &si_incoming, &slen)) == -1)
	{
		printf("recvfrom returned -1\n");
	}
	printf("recv_len = %d\n",recv_len);
	printf("Read Buffer -!-!-!-!-!-!-!-\n");
	for(int j=0;j<recv_len;j++) printf("%2.2hhX|",read_buffer[j]);
	printf("\n");
	if(recv_len>0)
	{
		if(read_buffer[0]!=H1_def||read_buffer[1]!=H2_def||computeChecksum(read_buffer, rbuf_size)!=(read_buffer[rbuf_size-1] & 0xff)||recv_len!=rbuf_size)
		{
			printf("Computed checksum: %2X\n", computeChecksum(read_buffer, rbuf_size));
			printf("Received checksum: %2X\n", read_buffer[rbuf_size-1]);
			printf("Received checksum: %2X\n", read_buffer[recv_len-1]);
			printf("rbuf_size %2i\n", rbuf_size);
			printf("BAD PACKET\n");
			//det.HSM_notify("BAD PACKET");
		}
		else
		{
			//print details of the client/peer and the data received
			printf("Received packet from %s:%d\n", inet_ntoa(si_incoming.sin_addr), ntohs(si_incoming.sin_port));
			printf("Data: %s\n" , read_buffer);
			//det.HSM_good();
		}
	}
	else 
	{
		printf("ERROR: port_read error\n");
		//det.HSM_notify("PORT_READ ERROR");
		return_val = false;
	}
	return return_val;
}


bool UDP_Receiver_Port::port_write()
{
	printf("ERROR: call made to UDP_Receiver::port_write()");

	return false;
}

UDP_Receiver_Port::~UDP_Receiver_Port()
{
	//
	// Close UDP port
	//
	close(s);
	
}

