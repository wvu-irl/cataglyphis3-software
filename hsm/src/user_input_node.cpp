#include <ros/ros.h>
#include <hsm/UserInputInitStartup.h>
#include <hsm/UserInputReboot.h>
#include <hsm/UserInputLostLocation.h>
#include <hsm/UserInputLostNorth.h>
#include <hsm/UserInputLostNextState.h>
#include <hsm/UserInputLostWaypointIndex.h>
#include <hsm/UserInputLostSampleProps.h>
#include <navigation/NavFilterOut.h>
//#include <robot_control/MissionPlanningInfo.h>
//#include <robot_control/ExecStateMachineInfo.h>
#include <iostream>

#define PI 3.14159

using namespace std;

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
	void navCallback(const navigation::NavFilterOut::ConstPtr& msg);
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
		else cout << "Input type selection not valid. Please try again. Choose 1 for initial startup, 2 for reboot, or 3 for robot lost." << endl; // Undefined input type
		
		user_input.printCurrentStatus();
		
		cout << endl << "Perform user input again (0) or end program (1)?" << endl;
		cin >> user_input.end_program; cout << endl;
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
	nav_sub = nh.subscribe<navigation::NavFilterOut>("navigation/navigationfilterout/navigationfilterout",1,&User_Input::navCallback,this);
	//planning_sub = nh.subscribe<robot_control::MissionPlanningInfo>("control/missionplanninginfo/missionplanninginfo",1,&User_Input::planningCallback,this);
	//exec_sub = nh.subscribe<robot_control::ExecStateMachineInfo>("control/statemachineinfo/statemachineinfo",1,&User_Input::execCallback,this);
}

void User_Input::begin()
{
	cout << "Is this initial startup (1), a reboot, (2), or is the robot lost (3)?" << endl;
	cin >> input_type; cout << endl;
}

void User_Input::runInitStartup()
{
	cout << "Skip initialization? (0 or 1)" << endl;
	cin >> skip_init; cout << endl;
	if(skip_init==0)
	{
		cout << "What is the initial north angle in degrees?" << endl;
		cin >> north_angle_init; cout << endl; cout << endl;
		cout << "Is it a sunny day? (0 or 1)" << endl;
		cin >> sunny_day; cout << endl;
		cout << "Perform text detection? (0 or 1)" << endl;
		cin >> text_detection; cout << endl;
		cout << "Perform bias removal on forklift/reboot? (0 or 1)" << endl;
		cin >> bias_removal_forklift; cout << endl;
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
			cout << "Do you want to keep these offsets and end initialization (0), or run bias removal again (1)?" << endl;
			cin >> rerun_bias_removal; cout << endl;
			if(rerun_bias_removal==0) break;
			else initBiasRemoval();
		}
	}
	else {init_msg_out.bias_removal = 0; init_pub.publish(init_msg_out);}
	init_msg_out.begin_dead_reckoning = 1;
	init_pub.publish(init_msg_out);
	cout << "Initialization ended." << endl << endl;
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
	cout << "p1 offset = " << p1_offset_in << endl;
	cout << "q1 offset = " << q1_offset_in << endl;
	cout << "r1 offset = " << r1_offset_in << endl;
	cout << "p2 offset = " << p2_offset_in << endl;
	cout << "q2 offset = " << q2_offset_in << endl;
	cout << "r2 offset = " << r2_offset_in << endl;
	cout << "p3 offset = " << p3_offset_in << endl;
	cout << "q3 offset = " << q3_offset_in << endl;
	cout << "r3 offset = " << r3_offset_in << endl << endl;
	init_msg_out.bias_removal = 0;
	init_pub.publish(init_msg_out);
}

void User_Input::runReboot()
{
	runInitStartup();
	cout << "What is the distance to the homing beacon in meters?" << endl;
	cin >> distance; cout << endl;
	cout << "What is the distance uncertainty in meters?" << endl;
	cin >> distance_unc; cout << endl;
	cout << "What is the bearing in degrees?" << endl;
	cin >> bearing; cout << endl;
	cout << "What is the bearing uncertainty in degrees?" << endl;
	cin >> bearing_unc; cout << endl;
	cout << "What is the heading in degrees?" << endl;
	cin >> heading; cout << endl;
	cout << "What is the heading uncertainty in degrees?" << endl;
	cin >> heading_unc; cout << endl;
	cout << "What is the north angle in degrees?" << endl;
	cin >> north_angle; cout << endl;
	cout << "What is the north angle uncertainty in degrees?" << endl;
	cin >> north_angle_unc; cout << endl;
	cout << "What should the next mission planning state be? (2 = calc_leave_home, 5 = calc_return_home, 8 = precached_search, 10 = sample_predef_search)" << endl;
	cin >> next_state; cout << endl;
	cout << "Which waypoint index should search start at?" << endl;
	cin >> waypoint_index; cout << endl;
	cout << "Precached acquired? (0 or 1)" << endl;
	cin >> precached_acquired; cout << endl;
	cout << "Samples collected count?" << endl;
	cin >> samples_collected_count; cout << endl;
	cout << "Is the robot possessing a sample? (0 or 1)" << endl;
	cin >> possessing_sample; cout << endl;
	cout << "What position should the grabber slides be set to? (1000 = open, -900 = closed)" << endl;
	cin >> slide_pos_out; cout << endl;
	
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
	cout << "Do you want to input robot location? (0 or 1)" << endl;
	cin >> input_location; cout << endl;
	if(input_location)
	{
		cout << "What is the distance to the homing beacon in meters?" << endl;
		cin >> distance; cout << endl;
		cout << "What is the distance uncertainty in meters?" << endl;
		cin >> distance_unc; cout << endl;
		cout << "What is the bearing in degrees?" << endl;
		cin >> bearing; cout << endl;
		cout << "What is the bearing uncertainty in degrees?" << endl;
		cin >> bearing_unc; cout << endl;
		cout << "What is the heading in degrees?" << endl;
		cin >> heading; cout << endl;
		cout << "What is the heading uncertainty in degrees?" << endl;
		cin >> heading_unc; cout << endl;
		lost_location_msg_out.x_position = distance*cos(bearing*PI/180.0);
		lost_location_msg_out.y_position = distance*sin(bearing*PI/180.0);
		arc_unc = (distance+distance_unc)*bearing_unc*PI/180.0;
		if(arc_unc>distance_unc) {reboot_msg_out.x_position_unc = arc_unc; reboot_msg_out.y_position_unc = arc_unc;}
		else {reboot_msg_out.x_position_unc = distance_unc; reboot_msg_out.y_position_unc = distance_unc;}
		lost_location_msg_out.heading = heading;
		lost_location_msg_out.heading_unc = heading_unc;
		lost_location_pub.publish(lost_location_msg_out);
	}
	cout << "Do you want to input the north angle? (0 or 1)" << endl;
	cin >> input_north; cout << endl;
	if(input_north)
	{
		cout << "What is the north angle in degrees?" << endl;
		cin >> north_angle; cout << endl;
		cout << "What is the north angle uncertainty in degrees?" << endl;
		cin >> north_angle_unc; cout << endl;
		lost_north_msg_out.north_angle = north_angle;
		lost_north_msg_out.north_angle_unc = north_angle_unc;
		lost_north_pub.publish(lost_north_msg_out);
	}
	cout << "Do you want to input the next mission planning state? (0 or 1)" << endl;
	cin >> input_next_state; cout << endl;
	if(input_next_state)
	{
		cout << "What should the next mission planning state be? (2 = calc_leave_home, 5 = calc_return_home, 8 = precached_search, 10 = sample_predef_search)" << endl;
		cin >> next_state; cout << endl;
		lost_next_state_msg_out.next_state = next_state;
		lost_next_state_pub.publish(lost_next_state_msg_out);
	}
	cout << "Do you want to input the waypoint index? (0 or 1)" << endl;
	cin >> input_waypoint_index; cout << endl;
	if(input_waypoint_index)
	{
		cout << "Which waypoint index should search start at?" << endl;
		cin >> waypoint_index; cout << endl;
		lost_waypoint_index_msg_out.waypoint_index = waypoint_index;
		lost_waypoint_index_pub.publish(lost_waypoint_index_msg_out);
	}
	cout << "Do you want to set sample properties (precached acquired, samples collected count, and possessing sample)? (0 or 1)" << endl;
	cin >> input_sample_props_acquired; cout << endl;
	if(input_sample_props_acquired)
	{
		cout << "Precached acquired? (0 or 1)" << endl;
		cin >> precached_acquired; cout << endl;
		cout << "Samples collected count?" << endl;
		cin >> samples_collected_count; cout << endl;
		cout << "Is the robot possessing a sample? (0 or 1)" << endl;
		cin >> possessing_sample; cout << endl;
		cout << "What position should the grabber slides be set to? (1000 = open, -900 = closed)" << endl;
	cin >> slide_pos_out; cout << endl;
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
	cout << endl << "-----------CURRENT STATUS-----------" << endl;
	cout << "x position = " << x_position_in << endl;
	cout << "y position = " << y_position_in << endl;
	cout << "distance = " << distance_in << endl;
	cout << "bearing = " << bearing_in << endl;
	cout << "heading = " << heading_in << endl;
	cout << "north angle = " << north_angle_in << endl;
	cout << "next mission planning state = " << next_state_in << endl;
	cout << "precached waypoint index = " << precached_waypoint_index_in << endl;
	cout << "sample waypoint index = " << sample_waypoint_index_in << endl;
	cout << "precached acquired = " << precached_acquired_in << endl;
	cout << "samples collected count = " << samples_collected_count_in << endl;
	cout << "possessing sample = " << possessing_sample_in << endl;
	cout << "p1 offset = " << p1_offset_in << endl;
	cout << "q1 offset = " << q1_offset_in << endl;
	cout << "r1 offset = " << r1_offset_in << endl;
	cout << "p2 offset = " << p2_offset_in << endl;
	cout << "q2 offset = " << q2_offset_in << endl;
	cout << "r2 offset = " << r2_offset_in << endl;
	cout << "p3 offset = " << p3_offset_in << endl;
	cout << "q3 offset = " << q3_offset_in << endl;
	cout << "r3 offset = " << r3_offset_in << endl << endl;
}

void User_Input::navCallback(const navigation::NavFilterOut::ConstPtr& msg)
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
