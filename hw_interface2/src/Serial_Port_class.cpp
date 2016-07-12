/**
 * Serial_Port_class.cpp
 * Implements serial port_open (via constructor), port_close (via destructor), port_read, and port_write methods 
 */
#include "Serial_Port_class.h"
#include <errno.h>
#include <stdint.h>
#include <ros/ros.h>

Serial_Port::Serial_Port(port_mode_t port_mode_input)
{
	/**
	 * Open serial port; save current port settings
	 * Define and set new settings
	 */
	checksum_type = _and;
	port_mode = port_mode_input;
	if(ros::param::get("MODEMDEVICE", MODEMDEVICE)==false) MODEMDEVICE = "/dev/ttyS0";
	if(ros::param::get("BAUDRATE", BAUDRATE)==false) BAUDRATE = 115200;
	if(port_mode==_fixed_length_packet) fd = open(MODEMDEVICE.c_str(), O_RDWR | O_NOCTTY/* | O_NONBLOCK */);
	else if(port_mode==_roboteq) fd = open(MODEMDEVICE.c_str(), O_RDWR | O_NOCTTY);
	else {ROS_ERROR("SERIAL PORT port_mode not specified");}
	if (fd <0) {perror(MODEMDEVICE.c_str()); ROS_ERROR("SERIAL PORT file descriptor open error"); }  //~~~provide message to HSM

	tcgetattr(fd,&oldtio); // save current port settings 

	if(BAUDRATE==115200) baud_rate = B115200;
	else if(BAUDRATE==57600) baud_rate = B57600;
	else if(BAUDRATE==19200) baud_rate = B19200;
	else baud_rate = B115200;

	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag = baud_rate | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;

	// set input mode (non-canonical, no echo,...) 
	if(port_mode==_fixed_length_packet) newtio.c_lflag = 0;
	else if(port_mode==_roboteq) newtio.c_lflag = ICANON;
	else newtio.c_lflag = 0;
 
	newtio.c_cc[VTIME]    = 1;   //Blocking read, timeout in ds !!!!!!// Nonblocking read - no time out
	newtio.c_cc[VMIN] = rbuf_size; // Nonblocking read - return whether or not a character is read
/*
	newtio.c_cc[VTIME]    = 2;   // inter-character timer ~~~ is this sufficient for full packet read modes? 
	newtio.c_cc[VMIN]     = 1;   // blocking read until 1 char is received 
*/

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd,TCSANOW,&newtio);
}

Serial_Port::Serial_Port()
{
	/**
	 * Open serial port; save current port settings
	 * Define and set new settings
	 */
	checksum_type = _not;
	port_mode = _fixed_length_packet;
	if(ros::param::get("MODEMDEVICE", MODEMDEVICE)==false) MODEMDEVICE = "/dev/ttyS0";
	if(ros::param::get("BAUDRATE", BAUDRATE)==false) BAUDRATE = 115200;
	if(port_mode==_fixed_length_packet) fd = open(MODEMDEVICE.c_str(), O_RDWR | O_NOCTTY/* | O_NONBLOCK */);
	else if(port_mode==_roboteq) fd = open(MODEMDEVICE.c_str(), O_RDWR | O_NOCTTY);
	else {ROS_ERROR("SERIAL PORT port_mode not specified"); exit(-1);}
	if (fd <0) {perror(MODEMDEVICE.c_str()); exit(-1); }  //~~~exit is probably not good here; provide message to HSM first

	tcgetattr(fd,&oldtio); // save current port settings 

	if(BAUDRATE==115200) baud_rate = B115200;
	else if(BAUDRATE==57600) baud_rate = B57600;
	else if(BAUDRATE==19200) baud_rate = B19200;
	else baud_rate = B115200;

	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag = baud_rate | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;

	// set input mode (non-canonical, no echo,...) 
	if(port_mode==_fixed_length_packet) newtio.c_lflag = 0;
	else if(port_mode==_roboteq) newtio.c_lflag = ICANON;
	else newtio.c_lflag = 0;
 
	newtio.c_cc[VTIME]    = 1;   //Blocking read, timeout in ds !!!!!!// Nonblocking read - no time out 
	newtio.c_cc[VMIN] = 0; // Nonblocking read - return whether or not a character is read
/*
	newtio.c_cc[VTIME]    = 2;   // inter-character timer ~~~ is this sufficient for full packet read modes? 
	newtio.c_cc[VMIN]     = 1;   // blocking read until 1 char is received 
*/

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd,TCSANOW,&newtio);
}  

bool Serial_Port::port_read()
{
	bool packet_read = false;
	bool checksum_valid;
	gettimeofday(&start_time, NULL);

	while(!(packet_read) && !(read_timeout()))
	{
		switch(port_mode)
		{
			case _fixed_length_packet:
				//ROS_INFO("read_mode = %d",read_mode);
				//read_mode = full_packet;
				switch(read_mode)
				{
					case single_char:
						switch(single_char_step)
						{
							case init_s:
								newtio.c_cc[VMIN] = 0;
								tcsetattr(fd,TCSANOW,&newtio);
								read_buffer[0] = 'A';  
								read_buffer[1] = 'z';
								//read_buffer[2] = 's';
								//read_buffer[3] = '2';
								//index = 4;
								index = 2;
								single_char_step = find_H1;
								read_mode = single_char;
								printf("INIT_S\n");
								break;
							case find_H1:
								res = read(fd,one_buff,1);
								if(res>0)
								{
									if(one_buff[0]==H1_def) single_char_step = find_H2;
								}						
								else single_char_step = find_H1;
								read_mode = single_char;
								printf("FIND_H1\n");
								break;
							case find_H2:
								res = read(fd,one_buff,1);
								//if(one_buff[0]==H2_def) single_char_step = find_H3;
								if(res>0)
								{
									if(one_buff[0]==H2_def) single_char_step = read_body;
								}
								else single_char_step = find_H1;
								read_mode = single_char;
								printf("FIND_H2\n");
								break;
							/*case find_H3: // This is probably going to be deleted
								res = read(fd,one_buff,1);
								if(res>0)
								{
									if(one_buff[0]==H3_def) single_char_step = read_body;
								}
								else single_char_step = find_H1;
								read_mode = single_char;
								printf("FIND_H3\n");
								break;*/
							case read_body:
								if(index>=rbuf_size) //~~~need to add checksum check and possibly validate packet before swapping modes
								{
									single_char_step = init_s;
									read_mode = full_packet;
									read_buffer[index]=0;	//terminate string so we can printf
															//~~~could be an overflow unless actual buffer is 	oversized 
									if(checksum_type==_and) checksum_value = computeChecksum(read_buffer, rbuf_size);
									else if(checksum_type==_not) checksum_value = computeChecksumNot(read_buffer, rbuf_size);
									ROS_INFO("calculated checksum_value = %X. packet checksum = %X",checksum_value,(read_buffer[rbuf_size-1] & 0xff));
									if(checksum_value!=(read_buffer[rbuf_size-1] & 0xff)) {packet_read = false; 
									/*det.HSM_notify("BAD PACKET");*/
									}
									else {packet_read = true; 
									/*det.HSM_good();*/
									}
									printf("SINGLE CHAR MODE COMPLETE\n");
									break;
								}
								res = read(fd,one_buff,1);
								printf("READ_BODY\n");
								if(res>0) 
								{
									read_buffer[index] = one_buff[0]; 
									index++; 
									single_char_step = read_body; 
									read_mode = single_char;
								}
								break;
						usleep(read_delay_usec);
						}
						break;
					case full_packet:
						switch(full_packet_step)
						{
							case init_f:
								newtio.c_cc[VMIN] = rbuf_size;   /* blocking read until full packet received */
								tcsetattr(fd,TCSANOW,&newtio);
								full_packet_step = run;
								read_mode = full_packet;
								printf("INIT_F\n");
								break;
							case run:
								/**
								 * Read data from the port - ~~~~~ currently a blocking read ~~~~~
								 */
								res = read(fd, read_buffer, rbuf_size);   /* returns after full packet is input */
								//printf("res = %d\n", res);
								printf("- - - - - *** read_buffer: ");
								for(int j=0;j<res;j++) printf("%2.2hhX|",read_buffer[j]);
								printf("\n");
								printf("RUN\n");
								if(checksum_type==_and) checksum_value = computeChecksum(read_buffer, rbuf_size);
								else if(checksum_type==_not) checksum_value = computeChecksumNot(read_buffer, rbuf_size);
								ROS_INFO("calculated checksum_value = %X. packet checksum = %X",checksum_value,(read_buffer[rbuf_size-1] & 0xff));
								if(res>0) //~~~consider more sophisticated logic if a packet read gets split
								{
									if(read_buffer[0]!=H1_def||read_buffer[1]!=H2_def||checksum_value!=(read_buffer[rbuf_size-1] & 0xff)||res!=rbuf_size)
																			//is it possible to timeout in the middle of a full packet read?
									{
										printf("BAD PACKET\n");
										//det.HSM_notify("BAD PACKET");
										/* debug: print character buffer */
										printf(":%s:%d\n", read_buffer, res);
										full_packet_step = init_f;
										read_mode = single_char;
										packet_read = false;
										tcflush(fd, TCIFLUSH);
									}
									else
									{
										read_buffer[res]=0; /* terminate string so we can printf */
										//det.HSM_good();
										full_packet_step = run;
										read_mode = full_packet;
										printf("FULL PACKET COMPLETE\n");
										packet_read = true;
										tcflush(fd, TCIFLUSH);
									}
								}
								break;
								usleep(read_delay_usec);

						}
						break;
				}
				break;
			case _roboteq:
				res = read(fd, read_buffer, rbuf_size);
				printf("- - - - - *** read_buffer: ");
				for(int j=0;j<res;j++) printf("%2.2c|",read_buffer[j]);
				printf("\n");
				if(read_buffer[0]!=H1_def||read_buffer[1]!=H2_def)
				{
					ROS_WARN("BAD PACKET");
					//det.HSM_notify("BAD PACKET"); // commented out temporarily
					packet_read = true; // Set to true because canonical read should always exit the internal while loop in this function
				}
				else
				{
					packet_read = true;
				}
				//usleep(read_delay_usec);
				break;
		}
				
	}
	return packet_read;
}

bool Serial_Port::simple_read(int read_length)
{
	bool packet_read;
	//printf("SIMPLE READ: read_length = %i\n",read_length);
	//printf("SIMPLE READ: rbuf_size = %i\n",rbuf_size);
	res = read(fd, read_buffer, read_length);
	printf("- - - - - *** read_buffer: ");
	for(int j=0;j<res;j++) printf("%2.2hhX|",read_buffer[j]);
	printf("\n");
	//printf("SIMPLE READ: res = %i\n",res);
	if(res==read_length) packet_read = true;
	else packet_read = false;
	return packet_read;
}

bool Serial_Port::read_timeout()
{
	long delta_usec;	
	gettimeofday(&current_time, NULL);
	delta_usec = (current_time.tv_sec - start_time.tv_sec)*1000000 + current_time.tv_usec - start_time.tv_usec;
    return (delta_usec >= timeout_usec);
}


bool Serial_Port::port_write()
{
	/**
	 * Write data to the port - 
	 */
	//ROS_INFO(" - - - - - - in Serial_Port::port_write()");
	//hexprint
	printf(" - - - - - - - write_buffer: "); 
	for(int i=0;i<wbuf_size;i++)
	{
		printf("%2.2hhX|",write_buffer[i]);
	}
	printf("\n"); 
	//--hexprint
	//ROS_INFO("wbuf_size = %d",wbuf_size);
	res = write(fd, write_buffer, wbuf_size);   /* write the number of characters in the  */
	//ROS_INFO(" - - - - - - write(fd, write_buffer, wbuf_size) returns %d",res);
}

int Serial_Port::flushBuffer()
{
    return tcflush(fd, TCIFLUSH);
}

/**********************************
 * destructor restores settings
 * and closes port
 */
Serial_Port::~Serial_Port()
{
	/**
	 * Restore prior port settings
	 */
	tcsetattr(fd,TCSANOW,&oldtio);
	/**
	 * Normal end to node execution
	 */
	close(fd);
}

