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

#include "user_input_mp_act_class.h"

User_Input_MP_Act::User_Input_MP_Act(planning_states_t* next_state_ptr_in, int* precached_waypoint_index_ptr_in, int* sample_waypoint_index_ptr_in) // Constructor
{
	next_state_ptr = next_state_ptr_in;
	precached_waypoint_index_ptr = precached_waypoint_index_ptr_in;
	sample_waypoint_index_ptr = sample_waypoint_index_ptr_in;
	reboot_sub = nh.subscribe<hsm::UserInputReboot>("hsm/userinput/reboot",1,&User_Input_MP_Act::rebootCallback,this);
	lost_next_state_sub = nh.subscribe<hsm::UserInputLostNextState>("hsm/userinput/nextstate",1,&User_Input_MP_Act::lostNextStateCallback,this);
	lost_waypoint_index_sub = nh.subscribe<hsm::UserInputLostWaypointIndex>("hsm/userinput/waypointindex",1,&User_Input_MP_Act::lostWaypointIndexSub,this);
}

void User_Input_MP_Act::rebootCallback(const hsm::UserInputReboot::ConstPtr& msg)
{
	*next_state_ptr = static_cast<planning_states_t>(msg->next_state);
	if(*next_state_ptr==precached_search) *precached_waypoint_index_ptr = msg->waypoint_index;
	else *sample_waypoint_index_ptr = msg->waypoint_index;
}

void User_Input_MP_Act::lostNextStateCallback(const hsm::UserInputLostNextState::ConstPtr& msg)
{
	*next_state_ptr = static_cast<planning_states_t>(msg->next_state);
}

void User_Input_MP_Act::lostWaypointIndexSub(const hsm::UserInputLostWaypointIndex::ConstPtr& msg)
{
	if(*next_state_ptr==precached_search) *precached_waypoint_index_ptr = msg->waypoint_index;
	else *sample_waypoint_index_ptr = msg->waypoint_index;
}
