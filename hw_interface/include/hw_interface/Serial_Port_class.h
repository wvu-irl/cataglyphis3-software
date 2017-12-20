/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef SERIAL_PORT_CLASS_H
#define SERIAL_PORT_CLASS_H
/**
 * Serial_Port_class.h
 */
//#include <sys/types.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <sys/time.h>
#include "Buffer_Interface_class.h"
#include "Port_Superclass.h"
//#include <hsm/HSM_Detector.h>

/**
 * Serial port configuration constants
 */
//#define BAUDRATE B115200
#define _POSIX_SOURCE 1           // POSIX compliant source
enum port_mode_t {_fixed_length_packet, _roboteq};
enum checksum_type_t {_and, _not};

class Serial_Port: public Buffer_Interface, public Port
{
public:
	//   Members
	/**
	 * fd - file descriptor, reference to opened port
	 * res - return value, number of characters read
	 * oldtio - prior port setttings
	 * newtio - new port settings
	 */
	std::string MODEMDEVICE;
	int BAUDRATE;
	speed_t baud_rate;
	int fd, res;
	struct termios oldtio, newtio;
	enum read_mode_t {single_char,full_packet} read_mode = single_char;
	enum single_char_step_t {init_s,find_H1,find_H2,find_H3,read_body} single_char_step = init_s;
	enum full_packet_step_t {init_f,run} full_packet_step = init_f;
	int index = 3;

	char one_buff[1];
//	HSM_Detector det;
	checksum_type_t checksum_type;
	uint8_t checksum_value = 0;
	//Methods
	Serial_Port(port_mode_t port_mode_input); //constructor - opens port
	Serial_Port();
	~Serial_Port();       					  //destructor - closes port
	bool port_read();
	bool port_write();
	bool simple_read(int read_length);
	int  flushBuffer();
private:
	//Members
	port_mode_t port_mode;
	struct timeval start_time, current_time;
	long timeout_usec = 500000;
	useconds_t read_delay_usec = 50000;

	//Methods
	bool read_timeout();
};

#endif /* SERIAL_PORT_CLASS_H */
