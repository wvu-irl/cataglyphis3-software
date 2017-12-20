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

#ifndef USER_INPUT_EXEC_ACT_CLASS_H
#define USER_INPUT_EXEC_ACT_CLASS_H
#include <ros/ros.h>
#include <hsm/UserInputReboot.h>
#include <hsm/UserInputLostSampleProps.h>
#include <hsm/UserInputInitStartup.h>
#include <hsm/UserInputLostNextState.h>
#include <hsm/UserInputLostWaypointIndex.h>
#include <hsm/UserInputLostLocation.h>
#include <hsm/UserInputLostNorth.h>

class User_Input_Exec_Act
{
public:
	// Members
	ros::NodeHandle nh;
	ros::Subscriber reboot_sub;
	ros::Subscriber lost_precached_acquired_sub;
	ros::Subscriber init_sub;
	ros::Subscriber lost_next_state_sub;
	ros::Subscriber lost_waypoint_index_sub;
	ros::Subscriber lost_location_sub;
	ros::Subscriber lost_north_sub;
	int* precached_acquired_ptr;
	int* samples_collected_count_ptr;
	bool* possessing_sample_ptr;
	int* travel_reset_init_ptr;
	int* text_detection_ptr;
	int* skip_init_ptr;
	int* slide_pos_out_ptr;
	// Methods
	User_Input_Exec_Act(int* precached_acquired_ptr_in, int* samples_collected_count_ptr_in, bool* possessing_sample_ptr_in, int* travel_reset_init_ptr_in, int* text_detection_ptr_in, int* skip_init_ptr_in, int* slide_pos_out_ptr_in); // Constructor
	void rebootCallback(const hsm::UserInputReboot::ConstPtr& msg);
	void lostSamplePropsCallback(const hsm::UserInputLostSampleProps::ConstPtr& msg);
	void initCallback(const hsm::UserInputInitStartup::ConstPtr& msg);
	void lostNextStateCallback(const hsm::UserInputLostNextState::ConstPtr& msg);
	void lostWaypointIndexSub(const hsm::UserInputLostWaypointIndex::ConstPtr& msg);
	void lostLocationCallback(const hsm::UserInputLostLocation::ConstPtr& msg);
	void lostNorthCallback(const hsm::UserInputLostNorth::ConstPtr& msg);
};

#endif /* USER_INPUT_EXEC_ACT_CLASS_H */
