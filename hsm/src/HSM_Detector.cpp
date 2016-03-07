#include "HSM_Detector.h"

HSM_Detector::HSM_Detector() // Constructor
{
	Init("");
}

HSM_Detector::HSM_Detector(std::string const & postfix) // detector-extension Constructor
{
	Init("/"+postfix);
}

HSM_Detector::HSM_Detector(ros::Duration TimeoutPeriod) // Constructor with timeout
{
	Init("");
	ros::NodeHandle nh;
	watchdogPeriod = TimeoutPeriod;
	watchdog =  nh.createTimer(watchdogPeriod, &HSM_Detector::HSM_timeout_callback, this);
	watchdog.stop(); // stop newly created timer until ready to time
}

HSM_Detector::~HSM_Detector() // Destructor
{
//	if(watchdog!=0){delete &watchdog;}
}

void HSM_Detector::Init(std::string postfix)
{
	ros::NodeHandle n;
	std::string node_name = ros::this_node::getName(); 
	pub = n.advertise<hsm::HSM_Detection>("HSM_Det/"+node_name+postfix,1);
	//ROS_INFO("HSM: %s publisher advertised.", pub.getTopic());
	msg.detector_node = node_name;
	error_reported = false;
//	watchdog = 0;
}

void HSM_Detector::HSM_notify(std::string error_type)
{
	if(watchdog!=0){watchdog_restart();}
	msg.message_type = error_type;
	pub.publish(msg);
	error_reported = true;
}

void HSM_Detector::HSM_good()
{
	if(watchdog!=0){watchdog_restart();}
	if (error_reported)
	{
		msg.message_type = "GOOD";
		pub.publish(msg);
		error_reported = false;
	}
}

void HSM_Detector::HSM_timeout_callback(const ros::TimerEvent& event)
{
	msg.message_type = "TIMEOUT";
	pub.publish(msg);
	error_reported = true;
	watchdog_restart();
}

void HSM_Detector::watchdog_start()
{
	watchdog.setPeriod(watchdogPeriod);
	watchdog.start();
}

void HSM_Detector::watchdog_restart()
{
	watchdog.stop();
	watchdog.setPeriod(watchdogPeriod);
	watchdog.start();	
}

void HSM_Detector::watchdog_setPeriod(ros::Duration newTimerPeriod)
{
	watchdogPeriod = newTimerPeriod;
}

