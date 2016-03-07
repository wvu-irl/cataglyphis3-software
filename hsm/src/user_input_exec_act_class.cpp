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
