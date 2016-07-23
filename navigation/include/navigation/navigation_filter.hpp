#ifndef NAVIGATION_FILTER_H
#define NAVIGATION_FILTER_H
#include <ros/ros.h>
#include <navigation/encoders_class.hpp>
#include <navigation/imu_class.hpp>
#include <navigation/filter_class.hpp>
#include <messages/ExecInfo.h>
#include <messages/NavFilterOut.h>
#include <messages/LidarFilterOut.h>
#include <hsm/user_input_nav_act_class.h>

#include <messages/NavFilterControl.h> //added for new User Interface -Matt G.

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

	ros::Subscriber sub_lidar;
	double homing_x;
	double homing_y;
	double homing_heading;
	double homing_found;

	Encoders encoders;
	IMU imu;
	Filter filter;
	Filter filter1;
	Filter filter2;
	Filter filter3;
	Filter init_filter;

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
	bool homing_updated = false;
	double drop_off_dist = 10000;
	int nav_status_output = 0;

	//this will not be the way to communicate the states, this is temporary
	enum state_t {_waiting, _forklift_drive, _run};
	const state_t state_waiting = _waiting;
	const state_t state_forklift_drive = _forklift_drive;
	const state_t state_run = _run;
	state_t state = state_waiting;
private:
	void getExecInfoCallback(const messages::ExecInfo::ConstPtr &msg);
	void getLidarFilterOutCallback(const messages::LidarFilterOut::ConstPtr &msg);

        //added for new User Interface -Matt G.
        bool navFilterControlServiceCallback(messages::NavFilterControl::Request &request, messages::NavFilterControl::Response &response);
};

#endif // NAVIGATION_FILTER_H