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

#include "user_input_exec_act_class.h"

User_Input_Exec_Act::User_Input_Exec_Act(int* precached_acquired_ptr_in, int* samples_collected_count_ptr_in, bool* possessing_sample_ptr_in, int* travel_reset_init_ptr_in, int* text_detection_ptr_in, int* skip_init_ptr_in, int* slide_pos_out_ptr_in) // Constructor
{
	precached_acquired_ptr = precached_acquired_ptr_in;
	samples_collected_count_ptr = samples_collected_count_ptr_in;
	possessing_sample_ptr = possessing_sample_ptr_in;
	travel_reset_init_ptr = travel_reset_init_ptr_in;
	text_detection_ptr = text_detection_ptr_in;
	skip_init_ptr = skip_init_ptr_in;
	slide_pos_out_ptr = slide_pos_out_ptr_in;
	reboot_sub = nh.subscribe<hsm::UserInputReboot>("hsm/userinput/reboot",1,&User_Input_Exec_Act::rebootCallback,this);
	lost_precached_acquired_sub = nh.subscribe<hsm::UserInputLostSampleProps>("hsm/userinput/sampleprops",1,&User_Input_Exec_Act::lostSamplePropsCallback,this);
	init_sub = nh.subscribe<hsm::UserInputInitStartup>("hsm/userinput/init",1,&User_Input_Exec_Act::initCallback,this);
	lost_next_state_sub = nh.subscribe<hsm::UserInputLostNextState>("hsm/userinput/nextstate",1,&User_Input_Exec_Act::lostNextStateCallback,this);
	lost_waypoint_index_sub = nh.subscribe<hsm::UserInputLostWaypointIndex>("hsm/userinput/waypointindex",1,&User_Input_Exec_Act::lostWaypointIndexSub,this);
	lost_location_sub = nh.subscribe<hsm::UserInputLostLocation>("hsm/userinput/location",1,&User_Input_Exec_Act::lostLocationCallback,this);
	lost_north_sub = nh.subscribe<hsm::UserInputLostNorth>("hsm/userinput/north",1,&User_Input_Exec_Act::lostNorthCallback,this);
}

void User_Input_Exec_Act::rebootCallback(const hsm::UserInputReboot::ConstPtr& msg)
{
	*precached_acquired_ptr = msg->precached_acquired;
	*samples_collected_count_ptr = msg->samples_collected_count;
	*possessing_sample_ptr = msg->possessing_sample;
	*slide_pos_out_ptr = msg->slide_pos_out;
	*travel_reset_init_ptr = 1;
}
void User_Input_Exec_Act::lostSamplePropsCallback(const hsm::UserInputLostSampleProps::ConstPtr& msg)
{
	*precached_acquired_ptr = msg->precached_acquired;
	*samples_collected_count_ptr = msg->samples_collected_count;
	*possessing_sample_ptr = msg->possessing_sample;
	*slide_pos_out_ptr = msg->slide_pos_out;
	*travel_reset_init_ptr = 1;
}

void User_Input_Exec_Act::initCallback(const hsm::UserInputInitStartup::ConstPtr& msg)
{
	*text_detection_ptr = msg->text_detection;
	*skip_init_ptr = msg->skip_init;
}

void User_Input_Exec_Act::lostNextStateCallback(const hsm::UserInputLostNextState::ConstPtr& msg)
{
	*travel_reset_init_ptr = 1;
}

void User_Input_Exec_Act::lostWaypointIndexSub(const hsm::UserInputLostWaypointIndex::ConstPtr& msg)
{
	*travel_reset_init_ptr = 1;
}

void User_Input_Exec_Act::lostLocationCallback(const hsm::UserInputLostLocation::ConstPtr& msg)
{
	*travel_reset_init_ptr = 1;
}

void User_Input_Exec_Act::lostNorthCallback(const hsm::UserInputLostNorth::ConstPtr& msg)
{
	*travel_reset_init_ptr = 1;
}
