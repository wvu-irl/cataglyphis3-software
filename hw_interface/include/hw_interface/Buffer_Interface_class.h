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

#ifndef BUFFER_INTERFACE_CLASS_H
#define BUFFER_INTERFACE_CLASS_H
#include<ros/ros.h>

enum buffer_RW_t {pktread, pktwrite};

class Buffer_Interface
{
public:
	// Members
	static bool reader;
	static char* read_buffer;
	static int rbuf_size;
	static char H1_def;
	static char H2_def;
	char H3_def = 's'; // Cannot be static
	char H4_def = 2;   // Cannot be static
	static bool writer;
	static char* write_buffer;
	static int wbuf_size;
	//Methods
	uint8_t computeChecksum(char* buf, int buf_len);
	uint8_t computeChecksumNot(char* buf, int buf_len);
};

bool Buffer_Interface::reader = false;
char* Buffer_Interface::read_buffer = 0; //set to null in order to detect unitialized buffers
int Buffer_Interface::rbuf_size = 0;
char Buffer_Interface::H1_def = 'A';
char Buffer_Interface::H2_def = 'z';
bool Buffer_Interface::writer = false;
char* Buffer_Interface::write_buffer = 0; //set to null in order to detect unitialized buffers
int Buffer_Interface::wbuf_size = 0;

uint8_t Buffer_Interface::computeChecksum(char* buf, int buf_len)
{
	uint8_t checksum_value;
	checksum_value = 0;
	for(int i=3; i<(buf_len-1); i++) // Use buf_len-1 because checksum byte itself should not be included in the checksum computation
	{
		checksum_value = (checksum_value + static_cast<uint8_t>(buf[i])) & 255;
		//ROS_ERROR("**!*!*!*!**!**!*** Intermediate checksum value = %d",checksum_value);
	}
	return checksum_value;
}

uint8_t Buffer_Interface::computeChecksumNot(char* buf, int buf_len)
{
	uint8_t checksum_value;
	checksum_value = 0;
	for(int i=2; i<(buf_len-1); i++) // Use buf_len-1 because checksum byte itself should not be included in the checksum computation
	{
		checksum_value = (checksum_value + static_cast<uint8_t>(buf[i]));
	}
	return ~checksum_value;
}

#endif /* BUFFER_INTERFACE_CLASS_H */
