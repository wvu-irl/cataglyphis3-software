#ifndef NAVIGATION_FILTER_H
#define NAVIGATION_FILTER_H
#include <ros/ros.h>
#include <navigation/encoders_class.hpp>
#include <navigation/imu_class.hpp>
#include <navigation/filter_class.hpp>
#include <messages/ExecInfo.h>
#include <messages/NavFilterOut.h>
//#include <hsm/user_input_nav_act_class.h>

class NavigationFilter
{
public:
	// Methods
	NavigationFilter();
	void update_time();
	void waiting();
	void forklift_drive();
	void run();
	// Members
	ros::NodeHandle nh;

	ros::Subscriber sub_exec;
	bool pause_switch;
	bool stopFlag;
	bool turnFlag;

	Encoders encoders;
	IMU imu;
	Filter filter;
	Filter filter1;
	Filter filter2;
	Filter filter3;
	Filter init_filter;
	//User_Input_Nav_Act user_input_nav_act;

	double current_time;
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
	bool first_pass = true;
	double drop_off_dist = 10000;
	int nav_status_output = 0;

	enum state_t {_waiting, _forklift_drive, _run};
	const state_t state_waiting = _waiting;
	const state_t state_forklift_drive = _forklift_drive;
	const state_t state_run = _run;
	state_t state = state_waiting;
private:
	void getExecInfoCallback(const messages::ExecInfo::ConstPtr &msg);
};

#endif // NAVIGATION_FILTER_H