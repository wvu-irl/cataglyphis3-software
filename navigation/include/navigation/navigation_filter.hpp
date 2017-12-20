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

#ifndef NAVIGATION_FILTER_H
#define NAVIGATION_FILTER_H
#include <ros/ros.h>
#include <navigation/encoders_class.hpp>
#include <navigation/imu_class.hpp>
#include <navigation/filter_class.hpp>
#include <messages/ExecInfo.h>
#include <messages/MissionPlanningInfo.h>
#include <messages/NavFilterOut.h>
#include <messages/LidarFilterOut.h>
#include <hsm/user_input_nav_act_class.h>

#include <messages/NavFilterControl.h> //added for new User Interface -Matt G.

#define DEG_2_RAD (PI/180.0)
#define RAD_2_DEG (180.0/PI)
enum state_t {_waiting, _forklift_drive, _run};

class NavigationFilter
{
	public:
		// Methods
		NavigationFilter();
		void update_time();
		void waiting(User_Input_Nav_Act user_input_nav_act);
		void forklift_drive(User_Input_Nav_Act user_input_nav_act);
	    void run(User_Input_Nav_Act user_input_nav_act);
	    //added for new User Interface -Matt G.
	    bool navFilterControlServiceCallback(messages::NavFilterControl::Request request, messages::NavFilterControl::Response response);
		// Members
		ros::NodeHandle nh;

	    ros::ServiceServer nav_control_server;
	    messages::NavFilterControl::Request latest_nav_control_request;

		ros::Subscriber sub_exec;
		bool pause_switch;
		bool stopFlag;
		bool turnFlag;

		ros::Subscriber sub_mission;
		bool possibly_lost = false;
		bool square_update = false;

		ros::Subscriber sub_lidar;
		double homing_x;
		double homing_y;
		double homing_heading;
		bool homing_found;
		double dull_x;
		double dull_y;
		double shiny_x;
		double shiny_y;
		double cylinder_std;
		short int registration_counter;
		short int registration_counter_prev;

		Encoders encoders;
		IMU imu;
		Filter filter;
		Filter filter1;
		Filter filter2;
		Filter filterS;
		Filter init_filter;

		double current_time;
		double stop_time;
		double dt = 0;

		const double PI = 3.14159265;
		const double G = 9.80665;
		int calibrate_counter = 0; //this used to be static in old program
		const double calibrate_time = 10.0; //60 seconds
		bool prev_stopped = true;
		bool collecting_accelerometer_data = false;
		bool collected_gyro_data = false;
		bool collected_gyro1_data = false;
		bool collected_gyro2_data = false;
		bool collected_gyro3_data = false;
		bool collected_gyro4_data = false;
		bool collected_gyro5_data = false;
		bool collected_gyro6_data = false;
		bool collected_gyroS_data = false;
		bool first_pass = true;
		bool homing_updated = false;
		bool stop_request = false;
		bool do_homing = false;
		double drop_off_dist = 10000;
		int nav_status_output = 0;
		double init_north_angle = 111.0*DEG_2_RAD;

		//this will not be the way to communicate the states, this is temporary

		const state_t state_waiting = _waiting;
		const state_t state_forklift_drive = _forklift_drive;
		const state_t state_run = _run;
		state_t state = state_waiting;
	private:
		void getExecInfoCallback(const messages::ExecInfo::ConstPtr &msg);
		void getMissionPlanningInfoCallback(const messages::MissionPlanningInfo::ConstPtr &msg);
		void getLidarFilterOutCallback(const messages::LidarFilterOut::ConstPtr &msg);

	    //added for new User Interface -Matt G.
	    bool navFilterControlServiceCallback(messages::NavFilterControl::Request &request, messages::NavFilterControl::Response &response);
};

#endif // NAVIGATION_FILTER_H
