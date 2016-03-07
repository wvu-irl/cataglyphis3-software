#ifndef ESCAPE_MONITOR_CLASS_H
#define ESCAPE_MONITOR_CLASS_H

#include <ros/ros.h>
#include <hsm/HSM_Detection.h>
#include <hsm/HSM_Action.h>
#include <navigation/NavFilterOut.h>
#include <robot_control/ExecStateMachineInfo.h>
#include "monitor_enums.h"
#include <roboteq_interface/encoder_data.h>

class Stall_Monitor
{
public:
	int monitorStall(float current);
	Stall_Monitor();
private:
	double init_time;
	unsigned int overcurrent_count = 0;
	int return_value = 0;
	const float stall_current_limit = 7.0; // amps
	const double stall_period_time = 0.25; // sec
	const unsigned int overcurrent_count_threshold = 300; // counts @ 100 Hz
};

class Escape_Monitor_class
{
public:
	//Members
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber nav_sub;
	ros::Subscriber left_roboteq_sub;
	ros::Subscriber right_roboteq_sub;
	ros::Subscriber exec_sub;
	float roll_angle = 0.0;
	float pitch_angle = 0.0;
	float tilt_angle = 0.0;
	hsm::HSM_Action msg_out;
	double current_time;
	double tilt_limit_start_time;
	const float tilt_limit = 25.0; // deg
	const float tilt_recover_limit = 20.0; // deg
	const int tilt_counter_limit = 10;
	const int tilt_recover_counter_limit = 10;
	int tilt_counter = 0;
	int tilt_recover_counter = 0;
	int init_count = 0;
	int turn_flag = 0;
	int traveling_to_wp = 0;
	int wheels_stalled = 0;
	const int wheel_stalled_limit = 3;
	int fl_stalled = 0;
	int ml_stalled = 0;
	int bl_stalled = 0;
	int fr_stalled = 0;
	int mr_stalled = 0;
	int br_stalled = 0;
	float fl_current = 0;
	float ml_current = 0;
	float bl_current = 0;
	float fr_current = 0;
	float mr_current = 0;
	float br_current = 0;
	monitor_states_t monitor_state;
	recovering_substates_t recovering_substate;
	ros::Timer timer;
	
	//Methods
	Escape_Monitor_class(); //constructor
	void service_monitor();
	void navCallback(const navigation::NavFilterOut::ConstPtr& msg_in);
	void leftRoboteqCallback(const roboteq_interface::encoder_data::ConstPtr& msg_in);
	void rightRoboteqCallback(const roboteq_interface::encoder_data::ConstPtr& msg_in);
	void execCallback(const robot_control::ExecStateMachineInfo::ConstPtr& msg_in);
	void recoveringTimerCallback(const ros::TimerEvent& event);
private:
	Stall_Monitor fl_stall_monitor;
	Stall_Monitor ml_stall_monitor;
	Stall_Monitor bl_stall_monitor;
	Stall_Monitor fr_stall_monitor;
	Stall_Monitor mr_stall_monitor;
	Stall_Monitor br_stall_monitor;
};
	
#endif /* ESCAPE_MONITOR_CLASS_H */
