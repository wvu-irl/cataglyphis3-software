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

#ifndef USER_INPUT_NAV_ACT_CLASS_H
#define USER_INPUT_NAV_ACT_CLASS_H
#include <ros/ros.h>
#include <hsm/UserInputInitStartup.h>
#include <hsm/UserInputReboot.h>
#include <hsm/UserInputLostLocation.h>
#include <hsm/UserInputLostNorth.h>

class User_Input_Nav_Act
{
public:
	// Members
	const double PI = 3.14159265;
	ros::NodeHandle nh;
	ros::Subscriber init_sub;
	ros::Subscriber reboot_sub;
	ros::Subscriber lost_location_sub;
	ros::Subscriber lost_north_sub;
	double* x_position_ptr;
	double* x_position_unc_ptr;
	double* y_position_ptr;
	double* y_position_unc_ptr;
	double* heading_ptr;
	double* heading_unc_ptr;
	double* north_angle_ptr;
	double* north_angle_unc_ptr;

	double north_angle_init;

	int skip_init;

	double* x_position1_ptr;
	double* x_position1_unc_ptr;
	double* y_position1_ptr;
	double* y_position1_unc_ptr;
	double* heading1_ptr;
	double* heading1_unc_ptr;
	double* north_angle1_ptr;
	double* north_angle1_unc_ptr;

	double* x_position2_ptr;
	double* x_position2_unc_ptr;
	double* y_position2_ptr;
	double* y_position2_unc_ptr;
	double* heading2_ptr;
	double* heading2_unc_ptr;
	double* north_angle2_ptr;
	double* north_angle2_unc_ptr;

	//float north_angle_init_unc;
	int sunny_day = 0;
	int bias_removal_forklift = 0;
	int begin_dead_reckoning = 0;
	// Methods
	User_Input_Nav_Act(double* x_position_ptr_in, double* x_position_unc_ptr_in, double* y_position_ptr_in, double* y_position_unc_ptr_in, double* heading_ptr_in, double* heading_unc_ptr_in, double* north_angle_ptr_in, double* north_angle_unc_ptr_in, double* x_position1_ptr_in, double* x_position1_unc_ptr_in, double* y_position1_ptr_in, double* y_position1_unc_ptr_in, double* heading1_ptr_in, double* heading1_unc_ptr_in, double* north_angle1_ptr_in, double* north_angle1_unc_ptr_in, double* x_position2_ptr_in, double* x_position2_unc_ptr_in, double* y_position2_ptr_in, double* y_position2_unc_ptr_in, double* heading2_ptr_in, double* heading2_unc_ptr_in, double* north_angle2_ptr_in, double* north_angle2_unc_ptr_in); // Constructor
	void initCallback(const hsm::UserInputInitStartup::ConstPtr& msg);
	void rebootCallback(const hsm::UserInputReboot::ConstPtr& msg);
	void lostLocationCallback(const hsm::UserInputLostLocation::ConstPtr& msg);
	void lostNorthCallback(const hsm::UserInputLostNorth::ConstPtr& msg);
};

#endif /* USER_INPUT_NAV_ACT_CLASS_H */
