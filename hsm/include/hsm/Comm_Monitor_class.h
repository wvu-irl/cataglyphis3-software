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

#ifndef COMM_MONITOR_CLASS_H
#define COMM_MONITOR_CLASS_H

#include <ros/ros.h>
#include <hsm/HSM_Detection.h>
#include <hsm/HSM_Action.h>
#include <messages/nb1_to_i7_msg.h>
#include <messages/nb2_3_to_i7_msg.h>
#include <string>
#include "Counter.h"
//#include "Timer.h"
#include "monitor_enums.h"

class Comm_Monitor_class
{
public:
	//Members
	ros::Publisher pub;
	ros::Subscriber udp_hsm_sub;
	ros::Subscriber serial_hsm_sub;
	ros::Subscriber udp_data_sub;
	ros::Subscriber serial_data_sub;
	ros::Subscriber nb2_data_sub;
	ros::Subscriber nb3_data_sub;
	ros::Timer timer; //should be an HSM Timer, but that needs a object qualification
//	Timer* timer;
	hsm::HSM_Detection detection_data; //local copy of detection msg data
	bool detection_flag;
	Counter detection_count;
	int detection_threshold;
	monitor_states_t monitor_state;
	recovering_substates_t recovering_substate;
	hsm::HSM_Action msg_out;
	double udp_callback_time;
	double serial_callback_time;
	double nb2_callback_time;
	double nb3_callback_time;
	double current_time;
	const double callback_timeout_time = 1.0;
	std::string recovering_error_type;
	int init_count;
	bool nb1_udp_go = false;
	bool nb1_serial_go = false;
	bool nb1_go = false;
	bool nb2_go = false;
	bool nb3_go = false;

	//Methods
	Comm_Monitor_class(); //constructor
	void service_monitor();
	void detectionCallback(const hsm::HSM_Detection::ConstPtr& msg_in);
	void recoveringTimerCallback(const ros::TimerEvent& event);
	void udpDataCallback(const messages::nb1_to_i7_msg::ConstPtr& msg_in);
	void serialDataCallback(const messages::nb1_to_i7_msg::ConstPtr& msg_in);
	void nb2DataCallback(const messages::nb2_3_to_i7_msg::ConstPtr& msg_in);
	void nb3DataCallback(const messages::nb2_3_to_i7_msg::ConstPtr& msg_in);
};

#endif /* COMM_MONITOR_CLASS_H */
