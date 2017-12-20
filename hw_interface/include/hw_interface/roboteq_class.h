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

#ifndef ROBOTEQ_CLASS_H
#define ROBOTEQ_CLASS_H

#include <ros/ros.h>
#include <Buffer_Interface_class.h>
#include <Serial_Port_class.h>
#include <messages/ActuatorOut.h>
#include <messages/encoder_data.h>
#include <messages/GrabberFeedback.h>
#include <drive_packet.h>
#include <grabber_packet.h>
#include <string>
#include <boost/lexical_cast.hpp>

class Roboteq_Class : public Buffer_Interface
{
public:
	// Members
	// Drive roboteq fields
	int motor_1_speed_cmd;
	int motor_2_speed_cmd;
	int motor_3_speed_cmd;
	int motor_1_encoder_count;
	int motor_2_encoder_count;
	int motor_3_encoder_count;
	int motor_1_encoder_count_prev;
	int motor_2_encoder_count_prev;
	int motor_3_encoder_count_prev;
	messages::encoder_data encoder_msg_out;
	// Grabber roboteq fields
	int slide_pos_cmd = 1000;
	int drop_pos_cmd = -1000;
	int slide_pos_cmd_prev = 1000;
	int drop_pos_cmd_prev = -1000;
	int grabber_stop_cmd = 0;
	int slide_status;
	int drop_status;
	int error_status;
	int slide_pos_status;
	int counter;
	int checksum;
	int computed_checksum;
	messages::GrabberFeedback grabber_msg_out;
	// Other fields
	int num_channels = 2;
	double callback_time;
	double left_encoder_time;
	double right_encoder_time;
	const float inter_command_delay = 0.001;
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;
	Serial_Port* port_ptr;
	char write_buffer_local[100];
	char read_buffer_local[100];
	Packet* packet_in_ptr;

	// Methods
	Roboteq_Class(); // constructor
	~Roboteq_Class(); // destructor
	void clearPluses();
	// Callbacks
	void frontCallback(const messages::ActuatorOut::ConstPtr& msg_in);
	void middleCallback(const messages::ActuatorOut::ConstPtr& msg_in);
	void backCallback(const messages::ActuatorOut::ConstPtr& msg_in);
	void leftCallback(const messages::ActuatorOut::ConstPtr& msg_in);
	void rightCallback(const messages::ActuatorOut::ConstPtr& msg_in);
	void grabberCallback(const messages::ActuatorOut::ConstPtr& msg_in);
	// Drive roboteq methods
	void setMotorSpeeds();
	void readEncoderValues();
	void setPrevValues();
	// Grabber roboteq methods
	void setGrabberCommands();
	void grabberStop();
	void readGrabberFeedback();
};

#endif /* ROBOTEQ_CLASS_H */
