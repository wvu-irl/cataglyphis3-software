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

#ifndef COMM_CLASS_H
#define COMM_CLASS_H

#include <ros/ros.h>
#include <string>

// Port includes
#include "Port_Superclass.h"
#include "Serial_Port_class.h"
#include "UDP_Sender_Port_class.h"
#include "UDP_Sender_Port_class_fault_test.h"
#include "UDP_Receiver_Port_class.h"

// Packet includes
#include "Buffer_Interface_class.h"
#include "Packet_Superclass.h"

#include "nb1_to_i7_packet.h"
#include "nb1_to_i7_serial_packet.h"
#include "i7_to_nb1_packet.h"
#include "nb2_3_to_i7_packet.h"

// Message includes

#include <messages/nb1_to_i7_msg.h>
#include <messages/nb1_to_i7_serial_msg.h>
#include <messages/i7_to_nb1_msg.h>
#include <messages/nb2_3_to_i7_msg.h>

// HSM includes
//#include "UDP_HSM_Act_class.h"
//#include "Serial_HSM_Act_class.h"

class Comm: public Buffer_Interface
{
public:
	// Members
	// ROS
	ros::NodeHandle nh;
	ros::Rate* loop_rate_ptr;
	ros::Publisher pub;
	Packet* packet_in_ptr;
	Packet* packet_out_ptr;
	Port* port_ptr;
	std::string node_type;
	//UDP_HSM_Act* UDP_act_ptr;
	//Serial_HSM_Act* Serial_act_ptr;
	int write_rate;
	bool publish_enabled;

	// Methods
	Comm(); // Constructor
	void run();
};

#endif /* COMM_CLASS_H */
