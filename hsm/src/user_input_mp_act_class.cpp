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
