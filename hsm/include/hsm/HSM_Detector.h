#ifndef HSM_DETECTOR_H
#define HSM_DETECTOR_H

#include <ros/ros.h>
#include <string>
#include <hsm/HSM_Detection.h>
//#include "Timer.h"


class HSM_Detector
{
private:
	// Members
	bool error_reported; //true if error reported and not returned to good
	// ROS
	ros::Timer watchdog;
	ros::Duration watchdogPeriod;
	// Methods
	void Init(std::string postfix);
	void HSM_timeout_callback(const ros::TimerEvent&);
	void watchdog_restart();

public:
	// Members
	std::string detector_node;
	std::string node_type;
	// ROS
	ros::Publisher pub;
	hsm::HSM_Detection msg;

	// Methods
	HSM_Detector(); // Constructor
	HSM_Detector(std::string const & postfix); // detector-extension Constructor
	HSM_Detector(ros::Duration TimeoutPeriod); // Constructor with timeout
	~HSM_Detector(); // Destructor
	void HSM_notify(std::string error_type);
	void HSM_good();
	void watchdog_setPeriod(ros::Duration);
	void watchdog_start();
};

#endif /* HSM_DETECTOR_H */
