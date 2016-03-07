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
