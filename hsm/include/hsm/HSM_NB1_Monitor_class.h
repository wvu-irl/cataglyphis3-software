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

#ifndef HSM_NB1_MONITOR_CLASS_H
#define HSM_NB1_MONITOR_CLASS_H

#include <ros/ros.h>
#include <hsm/HSM_Detection.h>
#include <hsm/HSM_Action.h>
#include <string>
#include "Counter.h"

enum monitor_states_t {Init_Monitor, Monitoring, Recovering};
enum recovering_substates_t {Commanding,Confirming,Waiting};

class HSM_NB1_Monitor_class
{
public:
	//Members
	ros::Publisher pub_action;
	ros::Subscriber sub_udp_detection;
	ros::Subscriber sub_serial_detection;
	ros::Timer timer;
	hsm::HSM_Detection udp_detection_data; //local copy of detection msg data
	hsm::HSM_Detection serial_detection_data; //local copy of detection msg data
	bool udp_detection_flag;
	bool serial_detection_flag;
	bool udp_missing_packet;
	bool serial_missing_packet;
	bool udp_bad_reads_flag;
	bool serial_bad_reads_flag;
	bool on_udp;
	Counter udp_bad_read_detection_count;
	Counter serial_bad_read_detection_count;
	Counter udp_timeout_detection_count;
	Counter serial_timeout_detection_count;
	Counter udp_node_restart_count;
	Counter serial_node_restart_count;
	int bad_read_detection_threshold;
	int simultaneous_bad_reads_threshold;
	int timeout_detection_threshold;
	int node_restart_threshold;
	monitor_states_t monitor_state;
	recovering_substates_t recovering_substate;
	hsm::HSM_Action msg_out;
	std::string recovering_error_type;
	int init_count;

	//Methods
	HSM_NB1_Monitor_class(); //constructor
	void service_monitor();
	void detectionCallback_udp(const hsm::HSM_Detection::ConstPtr& msg_in);
	void detectionCallback_serial(const hsm::HSM_Detection::ConstPtr& msg_in);
	void recoveringTimerCallback(const ros::TimerEvent& event);
};

#endif /* HSM_MONITOR_CLASS_H */
