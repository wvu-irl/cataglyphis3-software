#include "HSM_Detector.h"

HSM_Detector::HSM_Detector() // Constructor
{
	ros::NodeHandle n;
	if(ros::param::get("node_type", node_type)==false) node_type = "bad_node_type";
	pub = n.advertise<hsm::HSM_Detection>("HSM_Det/"+node_type,1);
	//ROS_INFO("HSM: %s publisher advertised.", pub.getTopic());
//	msg.consecutive_count = 0;
	msg.detector_type = ros::this_node::getName();
}

void HSM_Detector::HSM_notify(std::string error_type)
{
	msg.error_type = error_type;
//	msg.consecutive_count++; //this should be replaced with counter object accessible for reset and increment from the functional element
	pub.publish(msg);
}
