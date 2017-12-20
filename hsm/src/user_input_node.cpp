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

#include <ros/ros.h>
#include <hsm/UserInputInitStartup.h>
#include <hsm/UserInputReboot.h>
#include <hsm/UserInputLostLocation.h>
#include <hsm/UserInputLostNorth.h>
#include <hsm/UserInputLostNextState.h>
#include <hsm/UserInputLostWaypointIndex.h>
#include <hsm/UserInputLostSampleProps.h>
#include <messages/NavFilterOut.h>
//#include <robot_control/MissionPlanningInfo.h>
//#include <robot_control/ExecStateMachineInfo.h>
#include <iostream>

#define PI 3.14159

//using namespace std;

class User_Input
{
public:
	// Members
	// ROS members
	ros::NodeHandle nh;
	ros::Publisher init_pub;
	ros::Publisher reboot_pub;
	ros::Publisher lost_location_pub;
	ros::Publisher lost_north_pub;
	ros::Publisher lost_next_state_pub;
	ros::Publisher lost_waypoint_index_pub;
	ros::Publisher lost_sample_props_pub;
	ros::Subscriber nav_sub;
	ros::Subscriber planning_sub;
	ros::Subscriber exec_sub;
	ros::Rate* rate_ptr;
	hsm::UserInputInitStartup init_msg_out;
	hsm::UserInputReboot reboot_msg_out;
	hsm::UserInputLostLocation lost_location_msg_out;
	hsm::UserInputLostNorth lost_north_msg_out;
	hsm::UserInputLostNextState lost_next_state_msg_out;
	hsm::UserInputLostWaypointIndex lost_waypoint_index_msg_out;
	hsm::UserInputLostSampleProps lost_sample_props_msg_out;
	int input_type = 1; // 1 = initial startup, 2 = reboot, 3  = robot lost
	int end_program = 0;
	// initial startup members
	int skip_init = 0;
	float north_angle_init = 0.0; // deg
	int sunny_day = 0;
	int text_detection = 0;
	int bias_removal_forklift = 0;
	int rerun_bias_removal = 0;
	// reboot and lost members
	float distance = 0.0; // m
	float distance_unc = 0.0; // m
	float bearing = 0.0; // deg
	float bearing_unc = 0.0; // deg
	float arc_unc = 0.0;
	float heading = 0.0; // deg
	float heading_unc = 0.0; // deg
	float north_angle = 0.0; // deg
	float north_angle_unc = 0.0; // deg
	int next_state = 0;
	int waypoint_index = 0;
	int precached_acquired = 0;
	int samples_collected_count = 0;
	int possessing_sample = 0;
	int slide_pos_out = 1000;
	// computed members for reboot
	float x_position = 0.0;
	float y_position = 0.0;
	// Returned members from callbacks for nav
	float p1_offset_in = 0.0;
	float q1_offset_in = 0.0;
	float r1_offset_in = 0.0;
	float p2_offset_in = 0.0;
	float q2_offset_in = 0.0;
	float r2_offset_in = 0.0;
	float p3_offset_in = 0.0;
	float q3_offset_in = 0.0;
	float r3_offset_in = 0.0;
	float x_position_in = 0.0;
	float y_position_in = 0.0;
	float distance_in = 0.0;
	float bearing_in = 0.0;
	float heading_in = 0.0;
	float north_angle_in = 0.0;
	int nav_status_in = 0;
	// Returned members from callbacks for mission planning
	int next_state_in = 0;
	int precached_waypoint_index_in = 0;
	int sample_waypoint_index_in = 0;
	// Returned members from callbacks for exec state machine
	int precached_acquired_in = 0;
	int samples_collected_count_in = 0;
	int possessing_sample_in = 0;
	// Methods
	User_Input(); // constructor
	void begin();
	void runInitStartup();
	void runReboot();
	void runLost();
	void initBiasRemoval();
	void printCurrentStatus();
	void navCallback(const messages::NavFilterOut::ConstPtr& msg);
	//void planningCallback(const robot_control::MissionPlanningInfo::ConstPtr& msg);
	//void execCallback(const robot_control::ExecStateMachineInfo::ConstPtr& msg);
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "user_input_node");
	User_Input user_input;
	ros::spinOnce();
	ros::Duration(1).sleep();
	while(ros::ok())
	{
		user_input.printCurrentStatus();
		user_input.begin();
		ros::spinOnce();
		ros::Duration(1).sleep();
		if(user_input.input_type==1) user_input.runInitStartup(); // Run initial setup
		else if(user_input.input_type==2) user_input.runReboot(); // Run reboot
		else if(user_input.input_type==3) user_input.runLost(); // Run robot lost
                else std::cout << "Input type selection not valid. Please try again. Choose 1 for initial startup, 2 for reboot, or 3 for robot lost." << std::endl; // Undefined input type

		user_input.printCurrentStatus();

                std::cout << std::endl << "Perform user input again (0) or end program (1)?" << std::endl;
                std::cin >> user_input.end_program; std::cout << std::endl;
		if(user_input.end_program) break;
	}

	return 0;
}

User_Input::User_Input()
{
	rate_ptr = new ros::Rate(20); // 20 hz
	init_pub = nh.advertise<hsm::UserInputInitStartup>("hsm/userinput/init",1);
	reboot_pub = nh.advertise<hsm::UserInputReboot>("hsm/userinput/reboot",1);
	lost_location_pub = nh.advertise<hsm::UserInputLostLocation>("hsm/userinput/location",1);
	lost_north_pub = nh.advertise<hsm::UserInputLostNorth>("hsm/userinput/north",1);
	lost_next_state_pub = nh.advertise<hsm::UserInputLostNextState>("hsm/userinput/nextstate",1);
	lost_waypoint_index_pub = nh.advertise<hsm::UserInputLostWaypointIndex>("hsm/userinput/waypointindex",1);
	lost_sample_props_pub = nh.advertise<hsm::UserInputLostSampleProps>("hsm/userinput/sampleprops",1);
	nav_sub = nh.subscribe<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout",1,&User_Input::navCallback,this);
	//planning_sub = nh.subscribe<robot_control::MissionPlanningInfo>("control/missionplanninginfo/missionplanninginfo",1,&User_Input::planningCallback,this);
	//exec_sub = nh.subscribe<robot_control::ExecStateMachineInfo>("control/statemachineinfo/statemachineinfo",1,&User_Input::execCallback,this);
}

void User_Input::begin()
{
        std::cout << "Is this initial startup (1), a reboot, (2), or is the robot lost (3)?" << std::endl;
        std::cin >> input_type; std::cout << std::endl;
}

void User_Input::runInitStartup()
{
        std::cout << "Skip initialization? (0 or 1)" << std::endl;
        std::cin >> skip_init; std::cout << std::endl;
	if(skip_init==0)
	{
                std::cout << "What is the initial north angle in degrees?" << std::endl;
                std::cin >> north_angle_init; std::cout << std::endl; std::cout << std::endl;
                std::cout << "Is it a sunny day? (0 or 1)" << std::endl;
                std::cin >> sunny_day; std::cout << std::endl;
                std::cout << "Perform text detection? (0 or 1)" << std::endl;
                std::cin >> text_detection; std::cout << std::endl;
                std::cout << "Perform bias removal on forklift/reboot? (0 or 1)" << std::endl;
                std::cin >> bias_removal_forklift; std::cout << std::endl;
	}
	init_msg_out.skip_init = skip_init;
	init_msg_out.north_angle_init = north_angle_init;
	init_msg_out.sunny_day = sunny_day;
	init_msg_out.text_detection = text_detection;
	init_msg_out.begin_dead_reckoning = 0;
	if(bias_removal_forklift)
	{
		initBiasRemoval();
		while(ros::ok())
		{
                        std::cout << "Do you want to keep these offsets and end initialization (0), or run bias removal again (1)?" << std::endl;
                        std::cin >> rerun_bias_removal; std::cout << std::endl;
			if(rerun_bias_removal==0) break;
			else initBiasRemoval();
		}
	}
	else {init_msg_out.bias_removal = 0; init_pub.publish(init_msg_out);}
	init_msg_out.begin_dead_reckoning = 1;
	init_pub.publish(init_msg_out);
        std::cout << "Initialization ended." << std::endl << std::endl;
}
void User_Input::initBiasRemoval()
{
	init_msg_out.bias_removal = 1;
	init_pub.publish(init_msg_out);
	const double timeout_time = 30.0; // sec
	static double start_time = ros::Time::now().toSec();
	bool timeout = false;
	while(ros::ok())
	{
		ros::spinOnce();
		rate_ptr->sleep();
		if((ros::Time::now().toSec()-start_time)>=timeout_time) timeout = true;
		else timeout = false;
		if(nav_status_in!=0 || timeout) break;
	}
        std::cout << "p1 offset = " << p1_offset_in << std::endl;
        std::cout << "q1 offset = " << q1_offset_in << std::endl;
        std::cout << "r1 offset = " << r1_offset_in << std::endl;
        std::cout << "p2 offset = " << p2_offset_in << std::endl;
        std::cout << "q2 offset = " << q2_offset_in << std::endl;
        std::cout << "r2 offset = " << r2_offset_in << std::endl;
        std::cout << "p3 offset = " << p3_offset_in << std::endl;
        std::cout << "q3 offset = " << q3_offset_in << std::endl;
        std::cout << "r3 offset = " << r3_offset_in << std::endl << std::endl;
	init_msg_out.bias_removal = 0;
	init_pub.publish(init_msg_out);
}

void User_Input::runReboot()
{
	runInitStartup();
        std::cout << "What is the distance to the homing beacon in meters?" << std::endl;
        std::cin >> distance; std::cout << std::endl;
        std::cout << "What is the distance uncertainty in meters?" << std::endl;
        std::cin >> distance_unc; std::cout << std::endl;
        std::cout << "What is the bearing in degrees?" << std::endl;
        std::cin >> bearing; std::cout << std::endl;
        std::cout << "What is the bearing uncertainty in degrees?" << std::endl;
        std::cin >> bearing_unc; std::cout << std::endl;
        std::cout << "What is the heading in degrees?" << std::endl;
        std::cin >> heading; std::cout << std::endl;
        std::cout << "What is the heading uncertainty in degrees?" << std::endl;
        std::cin >> heading_unc; std::cout << std::endl;
        std::cout << "What is the north angle in degrees?" << std::endl;
        std::cin >> north_angle; std::cout << std::endl;
        std::cout << "What is the north angle uncertainty in degrees?" << std::endl;
        std::cin >> north_angle_unc; std::cout << std::endl;
        std::cout << "What should the next mission planning state be? (2 = calc_leave_home, 5 = calc_return_home, 8 = precached_search, 10 = sample_predef_search)" << std::endl;
        std::cin >> next_state; std::cout << std::endl;
        std::cout << "Which waypoint index should search start at?" << std::endl;
        std::cin >> waypoint_index; std::cout << std::endl;
        std::cout << "Precached acquired? (0 or 1)" << std::endl;
        std::cin >> precached_acquired; std::cout << std::endl;
        std::cout << "Samples collected count?" << std::endl;
        std::cin >> samples_collected_count; std::cout << std::endl;
        std::cout << "Is the robot possessing a sample? (0 or 1)" << std::endl;
        std::cin >> possessing_sample; std::cout << std::endl;
        std::cout << "What position should the grabber slides be set to? (1000 = open, -900 = closed)" << std::endl;
        std::cin >> slide_pos_out; std::cout << std::endl;

	reboot_msg_out.x_position = distance*cos(bearing*PI/180.0);
	reboot_msg_out.y_position = distance*sin(bearing*PI/180.0);
	arc_unc = (distance+distance_unc)*bearing_unc*PI/180.0;
	if(arc_unc>distance_unc) {reboot_msg_out.x_position_unc = arc_unc; reboot_msg_out.y_position_unc = arc_unc;}
	else {reboot_msg_out.x_position_unc = distance_unc; reboot_msg_out.y_position_unc = distance_unc;}
	reboot_msg_out.heading = heading;
	reboot_msg_out.heading_unc = heading_unc;
	reboot_msg_out.north_angle = north_angle;
	reboot_msg_out.north_angle_unc = north_angle_unc;
	reboot_msg_out.waypoint_index = waypoint_index;
	reboot_msg_out.precached_acquired = precached_acquired;
	reboot_msg_out.samples_collected_count = samples_collected_count;
	reboot_msg_out.next_state = next_state;
	reboot_msg_out.possessing_sample = possessing_sample;
	reboot_msg_out.slide_pos_out = slide_pos_out;
	reboot_pub.publish(reboot_msg_out);
	ros::spinOnce();
	ros::Duration(1).sleep();
}

void User_Input::runLost()
{
	int input_location = 0;
	int input_north = 0;
	int input_next_state = 0;
	int input_waypoint_index = 0;
	int input_sample_props_acquired = 0;
        std::cout << "Do you want to input robot location? (0 or 1)" << std::endl;
        std::cin >> input_location; std::cout << std::endl;
	if(input_location)
	{
                std::cout << "What is the distance to the homing beacon in meters?" << std::endl;
                std::cin >> distance; std::cout << std::endl;
                std::cout << "What is the distance uncertainty in meters?" << std::endl;
                std::cin >> distance_unc; std::cout << std::endl;
                std::cout << "What is the bearing in degrees?" << std::endl;
                std::cin >> bearing; std::cout << std::endl;
                std::cout << "What is the bearing uncertainty in degrees?" << std::endl;
                std::cin >> bearing_unc; std::cout << std::endl;
                std::cout << "What is the heading in degrees?" << std::endl;
                std::cin >> heading; std::cout << std::endl;
                std::cout << "What is the heading uncertainty in degrees?" << std::endl;
                std::cin >> heading_unc; std::cout << std::endl;
		lost_location_msg_out.x_position = distance*cos(bearing*PI/180.0);
		lost_location_msg_out.y_position = distance*sin(bearing*PI/180.0);
		arc_unc = (distance+distance_unc)*bearing_unc*PI/180.0;
		if(arc_unc>distance_unc) {reboot_msg_out.x_position_unc = arc_unc; reboot_msg_out.y_position_unc = arc_unc;}
		else {reboot_msg_out.x_position_unc = distance_unc; reboot_msg_out.y_position_unc = distance_unc;}
		lost_location_msg_out.heading = heading;
		lost_location_msg_out.heading_unc = heading_unc;
		lost_location_pub.publish(lost_location_msg_out);
	}
        std::cout << "Do you want to input the north angle? (0 or 1)" << std::endl;
        std::cin >> input_north; std::cout << std::endl;
	if(input_north)
	{
                std::cout << "What is the north angle in degrees?" << std::endl;
                std::cin >> north_angle; std::cout << std::endl;
                std::cout << "What is the north angle uncertainty in degrees?" << std::endl;
                std::cin >> north_angle_unc; std::cout << std::endl;
		lost_north_msg_out.north_angle = north_angle;
		lost_north_msg_out.north_angle_unc = north_angle_unc;
		lost_north_pub.publish(lost_north_msg_out);
	}
        std::cout << "Do you want to input the next mission planning state? (0 or 1)" << std::endl;
        std::cin >> input_next_state; std::cout << std::endl;
	if(input_next_state)
	{
                std::cout << "What should the next mission planning state be? (2 = calc_leave_home, 5 = calc_return_home, 8 = precached_search, 10 = sample_predef_search)" << std::endl;
                std::cin >> next_state; std::cout << std::endl;
		lost_next_state_msg_out.next_state = next_state;
		lost_next_state_pub.publish(lost_next_state_msg_out);
	}
        std::cout << "Do you want to input the waypoint index? (0 or 1)" << std::endl;
        std::cin >> input_waypoint_index; std::cout << std::endl;
	if(input_waypoint_index)
	{
                std::cout << "Which waypoint index should search start at?" << std::endl;
                std::cin >> waypoint_index; std::cout << std::endl;
		lost_waypoint_index_msg_out.waypoint_index = waypoint_index;
		lost_waypoint_index_pub.publish(lost_waypoint_index_msg_out);
	}
        std::cout << "Do you want to set sample properties (precached acquired, samples collected count, and possessing sample)? (0 or 1)" << std::endl;
        std::cin >> input_sample_props_acquired; std::cout << std::endl;
	if(input_sample_props_acquired)
	{
                std::cout << "Precached acquired? (0 or 1)" << std::endl;
                std::cin >> precached_acquired; std::cout << std::endl;
                std::cout << "Samples collected count?" << std::endl;
                std::cin >> samples_collected_count; std::cout << std::endl;
                std::cout << "Is the robot possessing a sample? (0 or 1)" << std::endl;
                std::cin >> possessing_sample; std::cout << std::endl;
                std::cout << "What position should the grabber slides be set to? (1000 = open, -900 = closed)" << std::endl;
        std::cin >> slide_pos_out; std::cout << std::endl;
		lost_sample_props_msg_out.precached_acquired = precached_acquired;
		lost_sample_props_msg_out.samples_collected_count = samples_collected_count;
		lost_sample_props_msg_out.possessing_sample = possessing_sample;
		lost_sample_props_msg_out.slide_pos_out = slide_pos_out;
		lost_sample_props_pub.publish(lost_sample_props_msg_out);
	}
	ros::spinOnce();
	ros::Duration(1).sleep();
}

void User_Input::printCurrentStatus()
{
	ros::spinOnce();
	ros::Duration(1).sleep();
        std::cout << std::endl << "-----------CURRENT STATUS-----------" << std::endl;
        std::cout << "x position = " << x_position_in << std::endl;
        std::cout << "y position = " << y_position_in << std::endl;
        std::cout << "distance = " << distance_in << std::endl;
        std::cout << "bearing = " << bearing_in << std::endl;
        std::cout << "heading = " << heading_in << std::endl;
        std::cout << "north angle = " << north_angle_in << std::endl;
        std::cout << "next mission planning state = " << next_state_in << std::endl;
        std::cout << "precached waypoint index = " << precached_waypoint_index_in << std::endl;
        std::cout << "sample waypoint index = " << sample_waypoint_index_in << std::endl;
        std::cout << "precached acquired = " << precached_acquired_in << std::endl;
        std::cout << "samples collected count = " << samples_collected_count_in << std::endl;
        std::cout << "possessing sample = " << possessing_sample_in << std::endl;
        std::cout << "p1 offset = " << p1_offset_in << std::endl;
        std::cout << "q1 offset = " << q1_offset_in << std::endl;
        std::cout << "r1 offset = " << r1_offset_in << std::endl;
        std::cout << "p2 offset = " << p2_offset_in << std::endl;
        std::cout << "q2 offset = " << q2_offset_in << std::endl;
        std::cout << "r2 offset = " << r2_offset_in << std::endl;
        std::cout << "p3 offset = " << p3_offset_in << std::endl;
        std::cout << "q3 offset = " << q3_offset_in << std::endl;
        std::cout << "r3 offset = " << r3_offset_in << std::endl << std::endl;
}

void User_Input::navCallback(const messages::NavFilterOut::ConstPtr& msg)
{
	p1_offset_in = msg->p1_offset;
	q1_offset_in = msg->q1_offset;
	r1_offset_in = msg->r1_offset;
	p2_offset_in = msg->p2_offset;
	q2_offset_in = msg->q2_offset;
	r2_offset_in = msg->r2_offset;
	p3_offset_in = msg->p3_offset;
	q3_offset_in = msg->q3_offset;
	r3_offset_in = msg->r3_offset;
	x_position_in = msg->x_position;
	y_position_in = msg->y_position;
	distance_in = hypot(msg->x_position,msg->y_position);
	bearing_in = msg->bearing;
	heading_in = msg->heading;
	north_angle_in = msg->north_angle;
	nav_status_in = msg->nav_status;
}

/*void User_Input::planningCallback(const robot_control::MissionPlanningInfo::ConstPtr& msg)
{
	next_state_in = msg->next_state;
	precached_waypoint_index_in = msg->precached_waypoint_index;
	sample_waypoint_index_in = msg->sample_waypoint_index;
}*/

/*void User_Input::execCallback(const robot_control::ExecStateMachineInfo::ConstPtr& msg)
{
	precached_acquired_in = msg->precached_acquired;
	samples_collected_count_in = msg->samples_collected_count;
	possessing_sample_in = msg->possessing_sample;
}*/
