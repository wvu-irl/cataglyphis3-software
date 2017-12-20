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

#include "Escape_Monitor_class.h"

Escape_Monitor_class::Escape_Monitor_class()
{
	emergencyEscapeTriggerClient = nh.serviceClient<messages::EmergencyEscapeTrigger>("/control/missionplanning/emergencyescapetrigger");
	nav_sub = nh.subscribe<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout",1,&Escape_Monitor_class::navCallback,this);
	left_roboteq_sub = nh.subscribe<messages::encoder_data>("roboteq/drivemotorin/left",1,&Escape_Monitor_class::leftRoboteqCallback,this);
	right_roboteq_sub = nh.subscribe<messages::encoder_data>("roboteq/drivemotorin/right",1,&Escape_Monitor_class::rightRoboteqCallback,this);
	exec_sub = nh.subscribe<messages::ExecInfo>("control/exec/info",1,&Escape_Monitor_class::execCallback,this);
	monitor_state = Init_Monitor;
	recovering_substate = Commanding;
	current_time = ros::Time::now().toSec();
	timer = nh.createTimer(ros::Duration(10),&Escape_Monitor_class::recoveringTimerCallback,this,true); //~~~should be an HSM Timer; callback needs object qualification
	timer.stop();
}

void Escape_Monitor_class::service_monitor()
{
	//ROS_INFO("-------------------------******************");
	tilt_angle = hypot(pitch_angle,roll_angle);
	switch(monitor_state)
	{
		case Init_Monitor:
			usleep(1000000); // Sleep 1 second to let ros services initialize
			emergencyEscapeTriggerSrv.request.escapeCondition = false;
			if(emergencyEscapeTriggerClient.call(emergencyEscapeTriggerSrv)) ROS_DEBUG("emergencyEscapeTrigger service call successful");
			else ROS_ERROR("emergencyEscapeTrigger service call unsuccessful");
			//std::cout << "Init - NO ESCAPE " << std::endl;
			init_count++;
			if(init_count>=3){monitor_state = Monitoring;}
			else monitor_state = Init_Monitor;
			break;
		case Monitoring:
			//ROS_INFO("tilt angle = %f",tilt_angle);
			if(tilt_angle>=tilt_limit) tilt_counter++;
			else tilt_counter = 0;
			//ROS_INFO("tilt_counter = %i",tilt_counter);
			if(exec_current_action==_driveGlobal || exec_current_action==_driveRelative)
			{
				if(turn_flag==0)
				{
					fl_stalled = fl_stall_monitor.monitorStall(fl_current);
					ml_stalled = ml_stall_monitor.monitorStall(ml_current);
					bl_stalled = bl_stall_monitor.monitorStall(bl_current);
					fr_stalled = fr_stall_monitor.monitorStall(fr_current);
					mr_stalled = mr_stall_monitor.monitorStall(mr_current);
					br_stalled = br_stall_monitor.monitorStall(br_current);
					wheels_stalled = fl_stalled + ml_stalled + bl_stalled + fr_stalled + mr_stalled + br_stalled;
				}
				else wheels_stalled = 0;
				if(tilt_counter>=tilt_counter_limit || wheels_stalled>=wheel_stalled_limit) monitor_state = Recovering;
				else monitor_state = Monitoring;
			}
			else monitor_state = Monitoring;
			break;
		case Recovering:
			switch(recovering_substate)
				{
					case Commanding:
						emergencyEscapeTriggerSrv.request.escapeCondition = true;
						if(emergencyEscapeTriggerClient.call(emergencyEscapeTriggerSrv)) ROS_DEBUG("emergencyEscapeTrigger service call successful");
						else ROS_ERROR("emergencyEscapeTrigger service call unsuccessful");
						monitor_state = Recovering;
						recovering_substate = Confirming;
						break;
					case Confirming:
						if(tilt_angle<=tilt_recover_limit) tilt_recover_counter++;
						else tilt_recover_counter = 0;
						if(tilt_recover_counter>=tilt_recover_counter_limit)
						{
							emergencyEscapeTriggerSrv.request.escapeCondition = false;
							if(emergencyEscapeTriggerClient.call(emergencyEscapeTriggerSrv)) ROS_DEBUG("emergencyEscapeTrigger service call successful");
							else ROS_ERROR("emergencyEscapeTrigger service call unsuccessful");
							monitor_state = Monitoring;
							recovering_substate = Commanding;
						}
						else
						{
							monitor_state = Recovering;
							recovering_substate = Confirming;
						}
						break;
				}
	}
	//ROS_INFO("monitor_state = %i",static_cast<uint8_t>(monitor_state));
	//ROS_INFO("recovering_substate = %i",static_cast<uint8_t>(recovering_substate));
	//ROS_INFO("******************-------------------------");
}

void Escape_Monitor_class::navCallback(const messages::NavFilterOut::ConstPtr& msg_in)
{
	pitch_angle = msg_in->pitch;
	roll_angle = msg_in->roll;
}

void Escape_Monitor_class::recoveringTimerCallback(const ros::TimerEvent& event)
{

	monitor_state = Monitoring;
	recovering_substate = Commanding;
}

void Escape_Monitor_class::leftRoboteqCallback(const messages::encoder_data::ConstPtr& msg_in)
{
	fl_current = msg_in->motor1_amps;
	ml_current = msg_in->motor2_amps;
	bl_current = msg_in->motor3_amps;
}
void Escape_Monitor_class::rightRoboteqCallback(const messages::encoder_data::ConstPtr& msg_in)
{
	fr_current = msg_in->motor1_amps;
	mr_current = msg_in->motor2_amps;
	br_current = msg_in->motor3_amps;
}

void Escape_Monitor_class::execCallback(const messages::ExecInfo::ConstPtr& msg_in)
{
	turn_flag = msg_in->turnFlag;
	exec_current_action = msg_in->actionDeque[0];
}

Stall_Monitor::Stall_Monitor()
{
	init_time = ros::Time::now().toSec();
}

int Stall_Monitor::monitorStall(float current)
{
	if(current>=stall_current_limit) {init_time = ros::Time::now().toSec(); overcurrent_count++;}
	if(overcurrent_count>=overcurrent_count_threshold) return_value = 1;
	else return_value = 0;
	if((ros::Time::now().toSec()-init_time)>=stall_period_time) overcurrent_count = 0;
	return return_value;
}
