#ifndef SAMPLE_SEARCH_H
#define SAMPLE_SEARCH_H
#include <ros/ros.h>
#include <computer_vision/capture_class.hpp> 
#include <messages/CVSearchCmd.h>
#include <messages/CVSamplesFound.h>
#include <string>
#include <fstream>

class SampleSearch
{
public:
	// Methods
	SampleSearch(); // Constructor
	bool searchForSamples(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res);
	// Members
	ros::NodeHandle nh;
	ros::Publisher search_pub;
	ros::ServiceServer searchForSamplesServ;
	messages::CVSamplesFound msg_cv_samples_found;
};

#endif // SAMPLE_SEARCH_H
