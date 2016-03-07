#ifndef USER_INPUT_MP_ACT_CLASS_H
#define USER_INPUT_MP_ACT_CLASS_H
#include <ros/ros.h>
#ifndef MISSION_PLANNING_STATES_H
#include <robot_control/mission_planning_states.h>
#endif /* MISSION_PLANNING_STATES_H */
#include <hsm/UserInputReboot.h>
#include <hsm/UserInputLostNextState.h>
#include <hsm/UserInputLostWaypointIndex.h>

class User_Input_MP_Act
{
public:
	// Members
	ros::NodeHandle nh;
	ros::Subscriber reboot_sub;
	ros::Subscriber lost_next_state_sub;
	ros::Subscriber lost_waypoint_index_sub;
	planning_states_t* next_state_ptr;
	int* precached_waypoint_index_ptr;
	int* sample_waypoint_index_ptr;
	// Methods
	User_Input_MP_Act(planning_states_t* next_state_ptr_in, int* precached_waypoint_index_ptr_in, int* sample_waypoint_index_ptr_in); // Constructor
	void rebootCallback(const hsm::UserInputReboot::ConstPtr& msg);
	void lostNextStateCallback(const hsm::UserInputLostNextState::ConstPtr& msg);
	void lostWaypointIndexSub(const hsm::UserInputLostWaypointIndex::ConstPtr& msg);
};

#endif /* USER_INPUT_MP_ACT_CLASS_H */
