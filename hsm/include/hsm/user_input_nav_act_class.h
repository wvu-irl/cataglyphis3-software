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
