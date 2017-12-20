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

/**
 * i7_to_nb1_packet.h
 * Header file with definition of WVU-NSRR Packet Type #2
 */
#include <messages/i7_to_nb1_msg.h>
#include "Buffer_Interface_class.h"
#include "Packet_Superclass.h"

class I7_To_NB1: public Buffer_Interface, public Packet
{
public:
	//Typedefs
	/**
	 * structure definition for WVU-NSRR packet 2
	 * (sent from i7 to nb1)
	 */
	struct i7_to_nb1_t {
		char H1;
		char H2;
		uint8_t H3;
		uint16_t counter;
		float x_position;
		float y_position;
		float heading;
		float north_angle;
		uint16_t waypoint_index;
		uint8_t waypoint_list_type;
		uint8_t samples_collected;
		uint8_t init_complete;
		uint8_t using_serial;
		uint8_t checksum;
	} __attribute__((packed));

	//Members
	/**
	 * union for common memory block for serial comm input buffer
	 * and the packet structure variables
	 */
	union {
		char buf[100]; /* size buffer >3X data size */
		i7_to_nb1_t pkt;
	};
	messages::i7_to_nb1_msg msg;

	//Methods
	I7_To_NB1(buffer_RW_t buffer_RW); //constructor to initialize Buffer_Interface
	void unpackMsg();
	void packMsg(const messages::i7_to_nb1_msg::ConstPtr& msg);
	void subscribeMsg();
	void publishMsg(ros::Publisher* pub_ptr);
	void debug_print_packet_data(int res);

};
