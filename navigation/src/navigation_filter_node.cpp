#include <ros/ros.h>
#include <stdint.h>
//#include <roboteq_interface/encoder_data.h>
#include <messages/NavFilterOut.h>
#include <navigation/encoders_class.hpp>
#include <navigation/filter_class.hpp>
//#include "sun_class.hpp"
#include <navigation/imu_class.hpp>
//#include "computer_vision/ComputerVisionOut.h"
#include <armadillo>
//#include "ss_class.hpp"
//#include <robot_control/bit_utils.h>
//#include <robot_control/nav_states.h>
//#include <robot_control/vision_states.h>
//#include <robot_control/ExecStateMachineInfo.h>
#include <messages/nb1_to_i7_msg.h>
#include <messages/nb2_3_to_i7_msg.h>
//#include <navigation/BarScanOut.h>
//#include <navigation/ObjectAvoidanceOut.h>
//#include <patch.hpp>
//#include <hsm/user_input_nav_act_class.h>
//#include <hsm/MasterStatus.h>

using namespace std;

class StateMachine
{
private:
	ros::Subscriber sub_sm;
	ros::NodeHandle node;
	void getStateMachineCallback(const robot_control::ExecStateMachineInfo::ConstPtr &msg)
	{
		this->turnFlag = msg->turn_flag;
		this->stopFlag = msg->stop_flag;
		this->ranging_radio_enable = msg->ranging_radio_enable;
		this->nav_cmd = static_cast<nav_mode_t>(msg->nav_cmd);
		this->vision_cmd = static_cast<vision_mode_t>(msg->vision_cmd);
	}
public:
	StateMachine();
	vision_mode_t vision_cmd;
	short int turnFlag;
	short int stopFlag;
	short int ranging_radio_enable;
	nav_mode_t nav_cmd;
};

StateMachine::StateMachine()
{
	turnFlag = 0;
	stopFlag = 0;
	ranging_radio_enable = 0;
	nav_cmd = _no_update;
	sub_sm = node.subscribe("/control/statemachineinfo/statemachineinfo", 1, &StateMachine::getStateMachineCallback, this);
}

class Comm
{
private:
	ros::Subscriber sub_nb1;
	ros::Subscriber sub_nb2;
	ros::NodeHandle node;
	
	void getNB1Callback(const comm::nb1_to_i7_msg::ConstPtr &msg)
	{
		this->nb1_pause_switch = msg->pause_switch;
	}
	void getNB2Callback(const comm::nb2_3_to_i7_msg::ConstPtr &msg)
	{
		this->nb2_pause_switch = msg->pause_switch;
	}
public:
	Comm();
	int nb1_pause_switch;
	int nb2_pause_switch;
};

Comm::Comm()
{
	nb1_pause_switch = 0;
	nb2_pause_switch = 0;
	sub_nb1 = node.subscribe("comm/nb1in/nb1in",1,&Comm::getNB1Callback,this);
	sub_nb2 = node.subscribe("comm/nb2in/nb2in",1,&Comm::getNB2Callback,this);
}

class Hsm
{
private:
	ros::Subscriber sub_ms;
	ros::NodeHandle node;
	
	void getMasterStatusCallback(const hsm::MasterStatus::ConstPtr &msg)
	{
		this->nb1_go = msg->nb1_go;
		this->nb2_go = msg->nb2_go;
		this->nb3_go = msg->nb3_go;
	}
public:
	Hsm();
	bool nb1_go = false;
	bool nb2_go = false;
	bool nb3_go = false;
};

Hsm::Hsm()
{
	sub_ms = node.subscribe("hsm/masterstatus/masterstatus",1,&Hsm::getMasterStatusCallback,this);
}


class BarScan
{
private:
	ros::Subscriber sub_bar_scan;
	ros::NodeHandle node;

	void getBarScanCallback(const navigation::BarScanOut::ConstPtr &msg)
	{
		this->status = msg->bar_status;
		this->x = msg->x_position;
		this->y = msg->y_position;
		this->heading = msg->heading;
	}
public:
	BarScan();
	double x;
	double y;
	double heading;
	int status;
};

BarScan::BarScan()
{
	x=0;
	y=0;
	heading=0;
	status=0;
	sub_bar_scan = node.subscribe("/navigation/barscanout/barscanout", 1, &BarScan::getBarScanCallback, this);
}


class ObjectAvoidance
{
private:
	ros::Subscriber sub;
	ros::NodeHandle node;

	void getObjectAvoidanceCallback(const navigation::ObjectAvoidanceOut::ConstPtr &msg)
	{
		this->state = msg->collision_status;
		this->dropOffThresholdCounter = msg->dropOffThresholdCounter;
	}
public:
	ObjectAvoidance();
	int state; //0 clear, 1, 2, and 3 avoid, 4 reduce speed
	int dropOffThresholdCounter;
};

ObjectAvoidance::ObjectAvoidance()
{
	state=0;
	dropOffThresholdCounter=0;
	sub = node.subscribe("/navigation/objectavoidanceout/objectavoidanceout", 1, &ObjectAvoidance::getObjectAvoidanceCallback, this);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cataglyphis_filter_node");
	ROS_INFO("cataglyphis_filter_node running...");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<navigation::NavFilterOut>("navigation/navigationfilterout/navigationfilterout",1);
	
	Encoders encoders;
	IMU imu;
	StateMachine state_machine;
	Comm comm_obj;
	Hsm hsm_obj;
	BarScan bar_scan;
	ObjectAvoidance object_avoidance;
	Filter filter;
	Filter filter1;
	Filter filter2;
	Filter filter3;
	Filter init_filter;
	Sun sun;
	ss_class ss;
	Leading_Edge_Latch nav_cmd_LEL;
	User_Input_Nav_Act user_input_nav_act(&filter.x, &filter.P_x, &filter.y, &filter.P_y, &filter.psi, &filter.P_psi, &filter.north_angle, &filter.P_north_angle, &filter1.x, &filter1.P_x, &filter1.y, &filter1.P_y, &filter1.psi, &filter1.P_psi, &filter1.north_angle, &filter1.P_north_angle, &filter2.x, &filter2.P_x, &filter2.y, &filter2.P_y, &filter2.psi, &filter2.P_psi, &filter2.north_angle, &filter2.P_north_angle, &filter3.x, &filter3.P_x, &filter3.y, &filter3.P_y, &filter3.psi, &filter3.P_psi, &filter3.north_angle, &filter3.P_north_angle);
	
	navigation::NavFilterOut msg_NavFilterOut;
	
	int nav_status_output = 0;

	encoders.set_wheel_radius(0.2286/2); // diameter = 0.2286 meters 9 inches
	encoders.set_counts_per_revolution(4476.16*1.062); //counts per revolution
	filter.initialize_states(0,0,3.14159265,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); 
	filter.set_imu_offset(0,0); //x_offset, y_offset
  
   	const double PI = 3.14159265;
	double begin = ros::Time::now().toSec(); //start timer
	double initial_time = begin;
	static int calibrate_counter = 0;
	const double calibrate_time = 10.0; // 60 sec
	double elapsed_time = 0.0;
	double G = 9.80665;
	enum state_t {_waiting, _forklift_drive, _run};
	static state_t state = _waiting;
	enum updates_t {_full_laser, _none};
	static updates_t updates = _none;
	bool prev_stopped = true;
	bool collecting_accelerometer_data = false;
	bool collected_gyro_data = false;
	bool collected_gyro1_data = false;
	bool collected_gyro2_data = false;
	bool collected_gyro3_data = false;
	int pause_switch = 255;
	bool first_pass = true;
	bool got_sun_here = false;
	double drop_off_dist = 10000;

	while(ros::ok())
	{
		elapsed_time = ros::Time::now().toSec()-initial_time;
		double dt = ros::Time::now().toSec()-begin;
		begin = ros::Time::now().toSec();
		if(hsm_obj.nb1_go && hsm_obj.nb2_go) pause_switch = 255*(comm_obj.nb1_pause_switch==255 || comm_obj.nb2_pause_switch==255);
    	else if(hsm_obj.nb1_go && !hsm_obj.nb2_go) pause_switch = comm_obj.nb1_pause_switch;
    	else if(!hsm_obj.nb1_go && hsm_obj.nb2_go) pause_switch = comm_obj.nb2_pause_switch;
    	else pause_switch = 255;
		ROS_INFO("*-*-*-*-*-*-*-*-*-*-*-*");
		ROS_INFO("State = %i",state);
		if(user_input_nav_act.skip_init==1) state = _run;
		switch(state)
		{
			case _waiting: // Take avg of imu data for set time
				if (user_input_nav_act.bias_removal_forklift!=1)
				{
					first_pass = true;
				}
				if (first_pass == true)
				{
					filter.clear_accelerometer_values();
					imu.clear_gyro1_values();
					imu.clear_gyro2_values();
					imu.clear_gyro3_values();
					prev_stopped = false;
					collecting_accelerometer_data = false;
					collected_gyro_data = false;
					collected_gyro1_data = false;
					collected_gyro2_data = false;
					collected_gyro3_data = false;
					first_pass = false;
				}
				imu.determine_new_data();
				imu.filter_imu_values();
				if ((fabs(sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az)-1)< 0.05 && sqrt((imu.p)*(imu.p)+(imu.q)*(imu.q)+(imu.r)*(imu.r))<0.01) && encoders.delta_distance == 0)
				{
					if (collecting_accelerometer_data)
					{
						if (filter.ax_values.size() > 200)
						{
							collecting_accelerometer_data = false;
							filter.roll_pitch_G_update();
							filter.clear_accelerometer_values();
						}
						else
						{
							filter.collect_accelerometer_data(imu.ax, imu.ay, imu.az);
						}
					}
					if (user_input_nav_act.bias_removal_forklift==1)
					{
						if (imu.p1_values.size() > 500 && collected_gyro1_data!=true)
						{
							collected_gyro1_data = true;
							imu.calculate_gyro1_offset();
							imu.set_gyro1_offset();
						}
						else if (collected_gyro1_data==true)
						{
							collected_gyro1_data = true;
						}
						else
						{
							imu.collect_gyro1_data();
							collected_gyro1_data = false;
						}
						if (imu.p2_values.size() > 500 && collected_gyro2_data!=true)
						{
							collected_gyro2_data = true;
							imu.calculate_gyro2_offset();
							imu.set_gyro2_offset();
						}
						else if (collected_gyro2_data==true)
						{
							collected_gyro2_data = true;
						}
						else
						{
							imu.collect_gyro2_data();
							collected_gyro2_data = false;
						}
						if (imu.p3_values.size() > 500 && collected_gyro3_data!=true)
						{
							collected_gyro3_data = true;
							imu.calculate_gyro3_offset();
							imu.set_gyro3_offset();
						}
						else if (collected_gyro3_data==true)
						{
							collected_gyro3_data = true;
						}
						else
						{
							imu.collect_gyro3_data();
							collected_gyro3_data = false;
						}
						if (collected_gyro1_data && collected_gyro2_data && collected_gyro3_data)
						{
							collected_gyro_data = true;
						}
						else
						{
							collected_gyro_data = false;
						}
					}
				}
				if (user_input_nav_act.bias_removal_forklift==1 && collected_gyro_data)
				{
					nav_status_output = 1;
				}
				else
				{
					nav_status_output = 0;
				}
				imu.set_prev_counters();
				if(user_input_nav_act.begin_dead_reckoning==1) 
				{
					state = _forklift_drive;
					init_filter.initialize_states(filter.phi,filter.theta,user_input_nav_act.north_angle_init,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					filter1.initialize_states(filter.phi,filter.theta,user_input_nav_act.north_angle_init,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					filter2.initialize_states(filter.phi,filter.theta,user_input_nav_act.north_angle_init,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					filter3.initialize_states(filter.phi,filter.theta,user_input_nav_act.north_angle_init,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					init_filter.initialize_variance(collected_gyro_data,2.0*PI/180.0); //performed bias removal, north_angle_unc
					first_pass = true;
				}
				else state = _waiting;
				calibrate_counter++;
				break;
			case _forklift_drive: 
				encoders.adjustEncoderWrapError();
				encoders.calculateWheelDistancesFromEncoders();
				encoders.calculateDeltaDistance6Wheels(0, 0); //turnFlag, stopFlag
				imu.determine_new_data();
				imu.filter_imu_values();
				if (imu.new_imu1!=0)
				{
					filter1.turning(imu.p1,imu.q1,imu.r1,imu.dt1);
				}
				else
				{
					filter1.blind_turning(imu.p1,imu.q1,imu.r1,imu.dt1);
				}
				if (imu.new_imu2!=0)
				{
					filter2.turning(imu.p2,imu.q2,imu.r2,imu.dt2);
				}
				else
				{
					filter2.blind_turning(imu.p2,imu.q2,imu.r2,imu.dt2);
				}
				if (imu.new_imu3!=0)
				{
					filter3.turning(imu.p3,imu.q3,imu.r3,imu.dt3);
				}
				else
				{
					filter3.blind_turning(imu.p3,imu.q3,imu.r3,imu.dt3);
				}

				init_filter.which_nb_to_keep(filter1.psi, filter2.psi, filter3.psi);
				if(init_filter.keep_nb == 1)
				{
					if (imu.new_imu1!=0)
					{
						init_filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else if (imu.new_imu2!=0)
					{
						init_filter.keep_nb = 2;
						init_filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else
					{
						init_filter.keep_nb = 3;
						init_filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
				}
				else if(init_filter.keep_nb == 2)
				{
					if (imu.new_imu2!=0)
					{
						init_filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else if (imu.new_imu1!=0)
					{
						init_filter.keep_nb = 1;
						init_filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else
					{
						init_filter.keep_nb = 3;
						init_filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
				}
				else
				{
					if (imu.new_imu3!=0)
					{
						init_filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else if (imu.new_imu1!=0)
					{
						init_filter.keep_nb = 1;
						init_filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
					else
					{
						init_filter.keep_nb = 2;
						init_filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
					}
				}

				if (fabs(fmod(init_filter.psi-filter.E_north_angle,2*PI))<filter.north_angle_thresh || fabs(fmod(init_filter.psi-filter.E_north_angle,2*PI))>2*PI-filter.north_angle_thresh)
				{
					filter.north_angle = init_filter.psi;
					filter.P_north_angle = init_filter.P_psi;
				}
				else
				{	
					filter.north_angle = filter.E_north_angle;
					filter.P_north_angle = filter.north_angle_thresh*filter.north_angle_thresh;
				}

				imu.set_prev_counters();
				if(pause_switch==0) 
				{
					state = _run; 
					filter.initialize_states(0,0,3.14159265,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); 
					filter1.initialize_states(0,0,3.14159265,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); 
					filter2.initialize_states(0,0,3.14159265,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); 
					filter3.initialize_states(0,0,3.14159265,1,0,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); 
					filter.set_imu_offset(0,0); //x_offset, y_offset
					if (fabs(fmod(init_filter.psi-filter.E_north_angle,2*PI))<filter.north_angle_thresh || fabs(fmod(init_filter.psi-filter.E_north_angle,2*PI))>2*PI-filter.north_angle_thresh)
					{
						filter.north_angle = init_filter.psi;
						filter.P_north_angle = init_filter.P_psi;
					}
					else
					{	
						filter.north_angle = filter.E_north_angle;
						filter.P_north_angle = filter.north_angle_thresh*filter.north_angle_thresh;
					}
				}
				else state = _forklift_drive;
				break;
			case _run: // Run nav filter
				state = _run;
				encoders.adjustEncoderWrapError();
				encoders.calculateWheelDistancesFromEncoders();
				encoders.calculateDeltaDistance6Wheels(0, 0); //turnFlag, stopFlag
				imu.determine_new_data();
				imu.filter_imu_values();
				
				// Predict Methods
				if(pause_switch==0) 
				{
					if (state_machine.turnFlag)
					{
						prev_stopped = false;
						collecting_accelerometer_data = false;
						collected_gyro_data = false;
						collected_gyro1_data = false;
						collected_gyro2_data = false;
						collected_gyro3_data = false;
						if (imu.new_imu1!=0)
						{
							filter1.turning(imu.p1,imu.q1,imu.r1,imu.dt1);
						}
						else
						{
							filter1.blind_turning(imu.p1,imu.q1,imu.r1,imu.dt1);
						}
						if (imu.new_imu2!=0)
						{
							filter2.turning(imu.p2,imu.q2,imu.r2,imu.dt2);
						}
						else
						{
							filter2.blind_turning(imu.p2,imu.q2,imu.r2,imu.dt2);
						}
						if (imu.new_imu3!=0)
						{
							filter3.turning(imu.p3,imu.q3,imu.r3,imu.dt3);
						}
						else
						{
							filter3.blind_turning(imu.p3,imu.q3,imu.r3,imu.dt3);
						}
					

						if(filter.keep_nb == 1)
						{
							filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						}
						else if(filter.keep_nb == 2)
						{
							filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						}
						else
						{
							filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						}


						filter.clear_accelerometer_values();
						got_sun_here = false;
						imu.clear_gyro1_values();
						imu.clear_gyro2_values();
						imu.clear_gyro3_values();
					}
					else if (state_machine.stopFlag)
					{
						if (!prev_stopped)
						{
							filter.which_nb_to_keep(filter1.psi, filter2.psi, filter3.psi);
							if(filter.keep_nb == 1)
							{
								if (imu.new_imu1!=0)
								{
									filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
								}
								else if (imu.new_imu2!=0)
								{
									filter.keep_nb = 2;
									filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
								}
								else
								{
									filter.keep_nb = 3;
									filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
								}
							}
							else if(filter.keep_nb == 2)
							{
								if (imu.new_imu2!=0)
								{
									filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
								}
								else if (imu.new_imu1!=0)
								{
									filter.keep_nb = 1;
									filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
								}
								else
								{
									filter.keep_nb = 3;
									filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
								}
							}
							else
							{
								if (imu.new_imu3!=0)
								{
									filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
								}
								else if (imu.new_imu1!=0)
								{
									filter.keep_nb = 1;
									filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
								}
								else
								{
									filter.keep_nb = 2;
									filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
								}
							}
							filter1.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
							filter2.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
							filter3.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y

						}


						if (!prev_stopped&&!collecting_accelerometer_data)
						{
							collecting_accelerometer_data = true;
						}
					
						if ((fabs(sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az)-1)< 0.05 && sqrt((imu.p)*(imu.p)+(imu.q)*(imu.q)+(imu.r)*(imu.r))<0.005) && encoders.delta_distance == 0)
						{
							if (collecting_accelerometer_data)
							{
								if (filter.ax_values.size() > 200)
								{
									collecting_accelerometer_data = false;
									filter.roll_pitch_G_update();
									filter.clear_accelerometer_values();
									filter1.initialize_states(filter.phi,filter.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
									filter2.initialize_states(filter.phi,filter.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
									filter3.initialize_states(filter.phi,filter.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
								}
								else
								{
									filter.collect_accelerometer_data(imu.ax, imu.ay, imu.az);
								}
							}
							if (state_machine.nav_cmd == _bias_removal)
							{
								if (imu.p1_values.size() > 500 && collected_gyro1_data!=true)
								{
									imu.calculate_gyro1_offset();
									if (imu.good_bias1 == true)
									{
										collected_gyro1_data = true;
										imu.set_gyro1_offset();
										filter1.Q_phi = 2.2847e-008;
										filter1.Q_theta = 2.2847e-008;
										filter1.Q_psi = 2.2847e-008;
									}
									else
									{
										imu.clear_gyro1_values();
									}
								}
								else if (collected_gyro1_data==true)
								{
									collected_gyro1_data = true;
								}
								else
								{
									imu.collect_gyro1_data();
									collected_gyro1_data = false;
								}
								if (imu.p2_values.size() > 500 && collected_gyro2_data!=true)
								{
									imu.calculate_gyro2_offset();
									if (imu.good_bias2 == true)
									{
										collected_gyro2_data = true;
										imu.set_gyro2_offset();
										filter2.Q_phi = 2.2847e-008;
										filter2.Q_theta = 2.2847e-008;
										filter2.Q_psi = 2.2847e-008;
									}
									else
									{
										imu.clear_gyro2_values();
									}
								}
								else if (collected_gyro2_data==true)
								{
									collected_gyro2_data = true;
								}
								else
								{
									imu.collect_gyro2_data();
									collected_gyro2_data = false;
								}
								if (imu.p3_values.size() > 500 && collected_gyro3_data!=true)
								{
									imu.calculate_gyro3_offset();
									if (imu.good_bias3 == true)
									{
										collected_gyro3_data = true;
										imu.set_gyro3_offset();
										filter1.Q_phi = 2.2847e-008;
										filter1.Q_theta = 2.2847e-008;
										filter1.Q_psi = 2.2847e-008;
									}
									else
									{
										imu.clear_gyro3_values();
									}
								}
								else if (collected_gyro3_data==true)
								{
									collected_gyro3_data = true;
								}
								else
								{
									imu.collect_gyro3_data();
									collected_gyro3_data = false;
								}
								if (collected_gyro1_data && collected_gyro2_data && collected_gyro3_data)
								{
									collected_gyro_data = true;
								}
								else
								{
									collected_gyro_data = false;
								}
							}
						}
						else
						{
							if (imu.new_imu1!=0)
							{
								filter1.dead_reckoning(imu.p1,imu.q1,imu.r1,encoders.delta_distance,imu.dt1);
							}
							else
							{
								filter1.blind_dead_reckoning(imu.p1,imu.q1,imu.r1,encoders.delta_distance,imu.dt1);
							}
							if (imu.new_imu2!=0)
							{
								filter2.dead_reckoning(imu.p2,imu.q2,imu.r2,encoders.delta_distance,imu.dt2);
							}
							else
							{
								filter2.blind_dead_reckoning(imu.p2,imu.q2,imu.r2,encoders.delta_distance,imu.dt2);
							}
							if (imu.new_imu3!=0)
							{
								filter3.dead_reckoning(imu.p3,imu.q3,imu.r3,encoders.delta_distance,imu.dt3);
							}
							else
							{
								filter3.blind_dead_reckoning(imu.p3,imu.q3,imu.r3,encoders.delta_distance,imu.dt3);
							}

							if(filter.keep_nb == 1)
							{
								filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
							}
							else if(filter.keep_nb == 2)
							{
								filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
							}
							else
							{
								filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
							}
						}
						prev_stopped = true;
					}
					else
					{
						prev_stopped = false;
						collecting_accelerometer_data = false;
						collected_gyro_data = false;
						collected_gyro1_data = false;
						collected_gyro2_data = false;
						collected_gyro3_data = false;
						filter.clear_accelerometer_values();
						got_sun_here = false;
						imu.clear_gyro1_values();
						imu.clear_gyro2_values();
						imu.clear_gyro3_values();
						if (imu.new_imu1!=0)
						{
							filter1.dead_reckoning(imu.p1,imu.q1,imu.r1,encoders.delta_distance,imu.dt1);
						}
						else
						{
							filter1.blind_dead_reckoning(imu.p1,imu.q1,imu.r1,encoders.delta_distance,imu.dt1);
						}
						if (imu.new_imu2!=0)
						{
							filter2.dead_reckoning(imu.p2,imu.q2,imu.r2,encoders.delta_distance,imu.dt2);
						}
						else
						{
							filter2.blind_dead_reckoning(imu.p2,imu.q2,imu.r2,encoders.delta_distance,imu.dt2);
						}
						if (imu.new_imu3!=0)
						{
							filter3.dead_reckoning(imu.p3,imu.q3,imu.r3,encoders.delta_distance,imu.dt3);
						}
						else
						{
							filter3.blind_dead_reckoning(imu.p3,imu.q3,imu.r3,encoders.delta_distance,imu.dt3);
						}
					

						if(filter.keep_nb == 1)
						{
							filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						}
						else if(filter.keep_nb == 2)
						{
							filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						}
						else
						{
							filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						}
					}

				}
				else 
				{
					prev_stopped = false;
					collecting_accelerometer_data = false;
					collected_gyro_data = false;
					collected_gyro1_data = false;
					collected_gyro2_data = false;
					collected_gyro3_data = false;
					filter.clear_accelerometer_values();
					imu.clear_gyro1_values();
					imu.clear_gyro2_values();
					imu.clear_gyro3_values();
					got_sun_here = false;
					if ((fabs(sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az)-1)< 0.05 && sqrt((imu.p)*(imu.p)+(imu.q)*(imu.q)+(imu.r)*(imu.r))<0.01) && encoders.delta_distance == 0)
					{
					}
					else
					{
						if (imu.new_imu1!=0)
						{
							filter1.dead_reckoning(imu.p1,imu.q1,imu.r1,encoders.delta_distance,imu.dt1);
						}
						else
						{
							filter1.blind_dead_reckoning(imu.p1,imu.q1,imu.r1,encoders.delta_distance,imu.dt1);
						}
						if (imu.new_imu2!=0)
						{
							filter2.dead_reckoning(imu.p2,imu.q2,imu.r2,encoders.delta_distance,imu.dt2);
						}
						else
						{
							filter2.blind_dead_reckoning(imu.p2,imu.q2,imu.r2,encoders.delta_distance,imu.dt2);
						}
						if (imu.new_imu3!=0)
						{
							filter3.dead_reckoning(imu.p3,imu.q3,imu.r3,encoders.delta_distance,imu.dt3);
						}
						else
						{
							filter3.blind_dead_reckoning(imu.p3,imu.q3,imu.r3,encoders.delta_distance,imu.dt3);
						}
						
						if(filter.keep_nb == 1)
						{
							filter.initialize_states(filter1.phi,filter1.theta,filter1.psi,filter1.x,filter1.y,filter1.P_phi,filter1.P_theta,filter1.P_psi,filter1.P_x,filter1.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						}
						else if(filter.keep_nb == 2)
						{
							filter.initialize_states(filter2.phi,filter2.theta,filter2.psi,filter2.x,filter2.y,filter2.P_phi,filter2.P_theta,filter2.P_psi,filter2.P_x,filter2.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						}
						else
						{
							filter.initialize_states(filter3.phi,filter3.theta,filter3.psi,filter3.x,filter3.y,filter3.P_phi,filter3.P_theta,filter3.P_psi,filter3.P_x,filter3.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						}
					}
				}


		

				// Update Methods
				// Find Available Updates

				updates = _none;

				if (bar_scan.status == 1)
				{	
					if (state_machine.stopFlag)
					{
						updates = _full_laser;
					}
				}
				
				

				// Implement Available Updates
				//updates = _none;
				switch (updates)
				{
					case _none:
						break;
					case _full_laser:
						filter.full_laser_update(bar_scan.x,bar_scan.y,bar_scan.heading);
						filter1.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						filter2.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						filter3.initialize_states(filter.phi,filter.theta,filter.psi,filter.x,filter.y,filter.P_phi,filter.P_theta,filter.P_psi,filter.P_x,filter.P_y); //phi,theta,psi,x,y,P_phi,P_theta,P_psi,P_x,P_y
						break;
				}

				if (state_machine.nav_cmd == _lidar_update)
				{
					nav_status_output=bar_scan.status;
				}
				else if (state_machine.nav_cmd == _bias_removal && collected_gyro_data)
				{
					nav_status_output = 1;
				}
				else
				{
					nav_status_output = 0;
				}
				
				if (state_machine.stopFlag&&user_input_nav_act.sunny_day&&!got_sun_here)
				{
					if(ss.isDataNew())
					{
					    if(ss.isValid())
					    {
							if (collecting_accelerometer_data == false)
							{
								if(ss.rad >800)
								{
									sun.get_calendar();
									sun.sun_position(-4.0, 42.2766, -71.8064, 0.0);
									filter.sun_north(sun.sun_azimuth,sun.sun_zenith,ss.angleX,ss.angleY);
									got_sun_here = true;
								}
							}
					    }
					}
					else
					{
					    ss.requestSSData();
					}
				}

				if(object_avoidance.dropOffThresholdCounter>=30)
				{
					drop_off_dist = 0.7;
				}
				else
				{
					drop_off_dist = 10000;
				}

				
				break;
		}

		filter.counter++;
		imu.set_prev_counters();
		msg_NavFilterOut.roll_rate = imu.p;
		msg_NavFilterOut.pitch_rate = imu.q;
		msg_NavFilterOut.yaw_rate = imu.r;
		msg_NavFilterOut.ax = imu.ax;
		msg_NavFilterOut.ay = imu.ay;
		msg_NavFilterOut.az = imu.az;
		msg_NavFilterOut.allSlipFlag=0;
		msg_NavFilterOut.counter=filter.counter;
		msg_NavFilterOut.nav_status = nav_status_output;
		msg_NavFilterOut.drop_off_distance = drop_off_dist;
		msg_NavFilterOut.p1_offset = imu.p1_offset;
		msg_NavFilterOut.q1_offset = imu.q1_offset;
		msg_NavFilterOut.r1_offset = imu.r1_offset;
		msg_NavFilterOut.p2_offset = imu.p2_offset;
		msg_NavFilterOut.q2_offset = imu.q2_offset;
		msg_NavFilterOut.r2_offset = imu.r2_offset;
		msg_NavFilterOut.p3_offset = imu.p3_offset;
		msg_NavFilterOut.q3_offset = imu.q3_offset;
		msg_NavFilterOut.r3_offset = imu.r3_offset;
		msg_NavFilterOut.update = updates;
		msg_NavFilterOut.calendar = patch::to_string(sun.calendar.year) + "-" + patch::to_string(sun.calendar.month) + "-" + patch::to_string(sun.calendar.day) + "-" + patch::to_string(sun.calendar.hour) + "-" + patch::to_string(sun.calendar.minute) + "-" + patch::to_string(sun.calendar.second);
		msg_NavFilterOut.x_position = filter.x;
		msg_NavFilterOut.y_position = filter.y;
		msg_NavFilterOut.roll = filter.phi*180/3.1415927;
		msg_NavFilterOut.pitch = filter.theta*180/3.1415927;
		msg_NavFilterOut.heading = filter.psi*180/3.1415927;
		msg_NavFilterOut.human_heading = fmod(filter.psi*180/3.1415927,360);
		msg_NavFilterOut.bearing = atan2(filter.y,filter.x)*180/3.1415927;
		msg_NavFilterOut.north_angle = filter.north_angle*180.0/PI; //128.0; // deg
		msg_NavFilterOut.velocity = encoders.delta_distance/dt;
		msg_NavFilterOut.sun_angle_x = ss.angleX;
		msg_NavFilterOut.sun_angle_y = ss.angleY;
		msg_NavFilterOut.sun_north_angle = filter.sun_north_angle*180/3.1415927;
		msg_NavFilterOut.estimated_zenith = filter.estimated_zenith*180/3.1415927;
		msg_NavFilterOut.sun_zenith = sun.sun_zenith;
		msg_NavFilterOut.sun_azimuth = sun.sun_azimuth;
		msg_NavFilterOut.dt = dt;
		msg_NavFilterOut.roll_init = init_filter.phi*180.0/PI;
		msg_NavFilterOut.pitch_init = init_filter.theta*180.0/PI;
		msg_NavFilterOut.heading_init = init_filter.psi*180.0/PI;
		pub.publish(msg_NavFilterOut);
		ros::spinOnce();
		ros::Duration(0.02).sleep();
	}

	return 0;
}
