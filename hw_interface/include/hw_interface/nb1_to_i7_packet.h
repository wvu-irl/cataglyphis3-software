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
 * nb1_to_i7_packet.h
 * Header file with definition of WVU-NSRR Packet Type #1
 */
#include <messages/nb1_to_i7_msg.h>
#include "Buffer_Interface_class.h"
#include "Packet_Superclass.h"

class NB1_To_I7: public Buffer_Interface, public Packet
{
public:
	//Typedefs
	/**
	 * structure definition for WVU-NSRR packet 1
	 * (sent from nb1 to i7)
	 */
	struct nb1_to_i7_t {
		char H1;
		char H2;
		uint8_t H3;
		uint16_t counter;
		uint32_t clock_reg_count;
		uint32_t clock_reg_reset_count;

		int32_t acc_x1;
		int32_t acc_y1;
		int32_t acc_z1;
		int32_t rate_p1;
		int32_t rate_q1;
		int32_t rate_r1;

		int32_t acc_x2;
		int32_t acc_y2;
		int32_t acc_z2;
		int32_t rate_p2;
		int32_t rate_q2;
		int32_t rate_r2;

		int32_t acc_x3;
		int32_t acc_y3;
		int32_t acc_z3;
		int32_t rate_p3;
		int32_t rate_q3;
		int32_t rate_r3;

        /*int32_t xda;
        int32_t yda;
        int32_t zda;
        int32_t xdv;
        int32_t ydv;
        int32_t zdv;*/

        //int16_t potValues[8];

        uint8_t imu_status; //bit shifted field containing info on IMU health
		uint8_t pause_switch;
		uint16_t main_loop_counter;
		uint8_t checksum;
	} __attribute__((packed));

	//Members
	/**
	 * union for common memory block for serial comm input buffer
	 * and the packet structure variables
	 */
	union {
		char buf[250]; /* size buffer >3X data size */
		nb1_to_i7_t pkt;
	};
	messages::nb1_to_i7_msg msg;

	//Methods
	NB1_To_I7(buffer_RW_t buffer_RW); //constructor to initialize Buffer_Interface
	void unpackMsg();
	void packMsg(const messages::nb1_to_i7_msg::ConstPtr& msg);
	void subscribeMsg();
	void publishMsg(ros::Publisher* pub_ptr);
	void debug_print_packet_data(int res);

};
