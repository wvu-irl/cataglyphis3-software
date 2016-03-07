#ifndef HSM_NB1_DETECTOR_H
#define HSM_NB1_DETECTOR_H

#include <ros/ros.h>
#include <string>
#include <hsm/HSM_Detection.h>
//#include "comm/src/ROS_Node_Superclass.h"


class HSM_NB1_Detector//: public ROS_Node_SC
{
public:
	// Members
	// ROS
	ros::Publisher pub;
	hsm::HSM_Detection msg; //standard detection message format
	std::string detector_type;
	std::string node_type;

	// Methods
	HSM_NB1_Detector(); // Constructor
	void HSM_NB1_notify(std::string error_type);
};

#endif /* HSM_NB1_DETECTOR_H */
