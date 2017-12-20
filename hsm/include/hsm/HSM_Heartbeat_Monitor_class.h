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

#ifndef HSM_HEARTBEAT_MONITOR_CLASS_H
#define HSM_HEARTBEAT_MONITOR_CLASS_H

#include <ros/ros.h>
#include <hsm/HSM_Detection.h>
#include <hsm/HSM_Action.h>
#include <string>
#include "Counter.h"
#include "monitor_enums.h"

#define NODE_STARTUP_TIMEOUT 30

class HSM_Heartbeat_Monitor_class
{
private:
	//Members
	std::string hb_node_name;
	int hb_node_Hz;
	ros::Duration hb_timeout_period;
	bool hb_node_died;
	ros::Timer heartbeat_timer;

	//Methods

public:
	//Members
	//--INPUTS: heartbeats
	ros::Subscriber sub_heartbeats;
	hsm::HSM_Detection heartbeat_data; //local copy of detection msg data

	//--OUTPUTS: node restart actions
	ros::Publisher pub_action;
	hsm::HSM_Action msg_out;

	//--monitor state machine variables
	monitor_states_t monitor_state;
	recovering_substates_t recovering_substate;
	std::string recovering_error_type;
	ros::Timer recovery_timer;
	std::string system_string;
	int monitor_trigger_count = 0;
	const int monitor_trigger_limit = 1;

	//Methods
	HSM_Heartbeat_Monitor_class(std::string hb_node_name, int hb_node_Hz); //constructor
	void service_monitor();
	void detectionCallback_heartbeat(const hsm::HSM_Detection::ConstPtr& msg_in);
	void recoveringTimerCallback(const ros::TimerEvent& event);
	void heartbeatTimerCallback(const ros::TimerEvent& event);
};

#endif /* HSM_HEARTBEAT_MONITOR_CLASS_H */
