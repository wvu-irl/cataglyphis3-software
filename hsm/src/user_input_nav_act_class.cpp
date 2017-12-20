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

#include "user_input_nav_act_class.h"

User_Input_Nav_Act::User_Input_Nav_Act(double* x_position_ptr_in, double* x_position_unc_ptr_in, double* y_position_ptr_in, double* y_position_unc_ptr_in, double* heading_ptr_in, double* heading_unc_ptr_in, double* north_angle_ptr_in, double* north_angle_unc_ptr_in, double* x_position1_ptr_in, double* x_position1_unc_ptr_in, double* y_position1_ptr_in, double* y_position1_unc_ptr_in, double* heading1_ptr_in, double* heading1_unc_ptr_in, double* north_angle1_ptr_in, double* north_angle1_unc_ptr_in, double* x_position2_ptr_in, double* x_position2_unc_ptr_in, double* y_position2_ptr_in, double* y_position2_unc_ptr_in, double* heading2_ptr_in, double* heading2_unc_ptr_in, double* north_angle2_ptr_in, double* north_angle2_unc_ptr_in) // Constructor
{
	x_position_ptr = x_position_ptr_in;
	x_position_unc_ptr = x_position_unc_ptr_in;
	y_position_ptr = y_position_ptr_in;
	y_position_unc_ptr = y_position_unc_ptr_in;
	heading_ptr = heading_ptr_in;
	heading_unc_ptr = heading_unc_ptr_in;
	north_angle_ptr = north_angle_ptr_in;
	north_angle_unc_ptr = north_angle_unc_ptr_in;

	x_position1_ptr = x_position1_ptr_in;
	x_position1_unc_ptr = x_position1_unc_ptr_in;
	y_position1_ptr = y_position1_ptr_in;
	y_position1_unc_ptr = y_position1_unc_ptr_in;
	heading1_ptr = heading1_ptr_in;
	heading1_unc_ptr = heading1_unc_ptr_in;
	north_angle1_ptr = north_angle1_ptr_in;
	north_angle1_unc_ptr = north_angle1_unc_ptr_in;

	x_position2_ptr = x_position2_ptr_in;
	x_position2_unc_ptr = x_position2_unc_ptr_in;
	y_position2_ptr = y_position2_ptr_in;
	y_position2_unc_ptr = y_position2_unc_ptr_in;
	heading2_ptr = heading2_ptr_in;
	heading2_unc_ptr = heading2_unc_ptr_in;
	north_angle2_ptr = north_angle2_ptr_in;
	north_angle2_unc_ptr = north_angle2_unc_ptr_in;

	skip_init = 0;

	init_sub = nh.subscribe<hsm::UserInputInitStartup>("hsm/userinput/init",1,&User_Input_Nav_Act::initCallback,this);
	reboot_sub = nh.subscribe<hsm::UserInputReboot>("hsm/userinput/reboot",1,&User_Input_Nav_Act::rebootCallback,this);
	lost_location_sub = nh.subscribe<hsm::UserInputLostLocation>("hsm/userinput/location",1,&User_Input_Nav_Act::lostLocationCallback,this);
	lost_north_sub = nh.subscribe<hsm::UserInputLostNorth>("hsm/userinput/north",1,&User_Input_Nav_Act::lostNorthCallback,this);
}

void User_Input_Nav_Act::initCallback(const hsm::UserInputInitStartup::ConstPtr& msg)
{
	skip_init = msg->skip_init;
	sunny_day = msg->sunny_day;
	bias_removal_forklift = msg->bias_removal;
	north_angle_init = (double)msg->north_angle_init*PI/180.0;
	begin_dead_reckoning = msg->begin_dead_reckoning;
}

void User_Input_Nav_Act::rebootCallback(const hsm::UserInputReboot::ConstPtr& msg)
{
	*x_position_ptr = (double)msg->x_position;
	*x_position_unc_ptr = pow((double)msg->x_position_unc,2);
	*y_position_ptr = (double)msg->y_position;
	*y_position_unc_ptr = pow((double)msg->y_position_unc,2);
	*heading_ptr = (double)msg->heading*PI/180.0;
	*heading_unc_ptr = pow((double)msg->heading_unc*PI/180.0,2);
	*north_angle_ptr = (double)msg->north_angle*PI/180.0;
	*north_angle_unc_ptr = pow((double)msg->north_angle_unc*PI/180.0,2);

	*x_position1_ptr = (double)msg->x_position;
	*x_position1_unc_ptr = pow((double)msg->x_position_unc,2);
	*y_position1_ptr = (double)msg->y_position;
	*y_position1_unc_ptr = pow((double)msg->y_position_unc,2);
	*heading1_ptr = (double)msg->heading*PI/180.0;
	*heading1_unc_ptr = pow((double)msg->heading_unc*PI/180.0,2);
	*north_angle1_ptr = (double)msg->north_angle*PI/180.0;
	*north_angle1_unc_ptr = pow((double)msg->north_angle_unc*PI/180.0,2);

	*x_position2_ptr = (double)msg->x_position;
	*x_position2_unc_ptr = pow((double)msg->x_position_unc,2);
	*y_position2_ptr = (double)msg->y_position;
	*y_position2_unc_ptr = pow((double)msg->y_position_unc,2);
	*heading2_ptr = (double)msg->heading*PI/180.0;
	*heading2_unc_ptr = pow((double)msg->heading_unc*PI/180.0,2);
	*north_angle2_ptr = (double)msg->north_angle*PI/180.0;
	*north_angle2_unc_ptr = pow((double)msg->north_angle_unc*PI/180.0,2);
}

void User_Input_Nav_Act::lostLocationCallback(const hsm::UserInputLostLocation::ConstPtr& msg)
{
	*x_position_ptr = (double)msg->x_position;
	*x_position_unc_ptr = pow((double)msg->x_position_unc,2);
	*y_position_ptr = (double)msg->y_position;
	*y_position_unc_ptr = pow((double)msg->y_position_unc,2);
	*heading_ptr = (double)msg->heading*PI/180.0;
	*heading_unc_ptr = pow((double)msg->heading_unc*PI/180.0,2);

	*x_position1_ptr = (double)msg->x_position;
	*x_position1_unc_ptr = pow((double)msg->x_position_unc,2);
	*y_position1_ptr = (double)msg->y_position;
	*y_position1_unc_ptr = pow((double)msg->y_position_unc,2);
	*heading1_ptr = (double)msg->heading*PI/180.0;
	*heading1_unc_ptr = pow((double)msg->heading_unc*PI/180.0,2);

	*x_position2_ptr = (double)msg->x_position;
	*x_position2_unc_ptr = pow((double)msg->x_position_unc,2);
	*y_position2_ptr = (double)msg->y_position;
	*y_position2_unc_ptr = pow((double)msg->y_position_unc,2);
	*heading2_ptr = (double)msg->heading*PI/180.0;
	*heading2_unc_ptr = pow((double)msg->heading_unc*PI/180.0,2);
}
void User_Input_Nav_Act::lostNorthCallback(const hsm::UserInputLostNorth::ConstPtr& msg)
{
	*north_angle_ptr = (double)msg->north_angle*PI/180.0;
	*north_angle_unc_ptr = pow((double)msg->north_angle_unc*PI/180.0,2);
}
