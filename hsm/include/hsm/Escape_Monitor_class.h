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

#ifndef ESCAPE_MONITOR_CLASS_H
#define ESCAPE_MONITOR_CLASS_H

#include <ros/ros.h>
#include <hsm/HSM_Detection.h>
#include <hsm/HSM_Action.h>
#include <messages/NavFilterOut.h>
#include <messages/ExecInfo.h>
#include <messages/EmergencyEscapeTrigger.h>
#include <robot_control/action_type_enum.h>
#include "monitor_enums.h"
#include <messages/encoder_data.h>

class Stall_Monitor
{
public:
	int monitorStall(float current);
	Stall_Monitor();
private:
	double init_time;
	unsigned int overcurrent_count = 0;
	int return_value = 0;
	const float stall_current_limit = 7.0; // amps
	const double stall_period_time = 0.25; // sec
	const unsigned int overcurrent_count_threshold = 300; // counts @ 100 Hz
};

class Escape_Monitor_class
{
public:
	//Members
	ros::NodeHandle nh;
	ros::ServiceClient emergencyEscapeTriggerClient;
	ros::Subscriber nav_sub;
	ros::Subscriber left_roboteq_sub;
	ros::Subscriber right_roboteq_sub;
	ros::Subscriber exec_sub;
	float roll_angle = 0.0;
	float pitch_angle = 0.0;
	float tilt_angle = 0.0;
	messages::EmergencyEscapeTrigger emergencyEscapeTriggerSrv;
	double current_time;
	double tilt_limit_start_time;
	const float tilt_limit = 25.0; // deg
	const float tilt_recover_limit = 20.0; // deg
	const int tilt_counter_limit = 10;
	const int tilt_recover_counter_limit = 10;
	int tilt_counter = 0;
	int tilt_recover_counter = 0;
	int init_count = 0;
	int turn_flag = 0;
	int exec_current_action = 0;
	int wheels_stalled = 0;
	const int wheel_stalled_limit = 3;
	int fl_stalled = 0;
	int ml_stalled = 0;
	int bl_stalled = 0;
	int fr_stalled = 0;
	int mr_stalled = 0;
	int br_stalled = 0;
	float fl_current = 0;
	float ml_current = 0;
	float bl_current = 0;
	float fr_current = 0;
	float mr_current = 0;
	float br_current = 0;
	monitor_states_t monitor_state;
	recovering_substates_t recovering_substate;
	ros::Timer timer;

	//Methods
	Escape_Monitor_class(); //constructor
	void service_monitor();
	void navCallback(const messages::NavFilterOut::ConstPtr& msg_in);
	void leftRoboteqCallback(const messages::encoder_data::ConstPtr& msg_in);
	void rightRoboteqCallback(const messages::encoder_data::ConstPtr& msg_in);
	void execCallback(const messages::ExecInfo::ConstPtr& msg_in);
	void recoveringTimerCallback(const ros::TimerEvent& event);
private:
	Stall_Monitor fl_stall_monitor;
	Stall_Monitor ml_stall_monitor;
	Stall_Monitor bl_stall_monitor;
	Stall_Monitor fr_stall_monitor;
	Stall_Monitor mr_stall_monitor;
	Stall_Monitor br_stall_monitor;
};

#endif /* ESCAPE_MONITOR_CLASS_H */
