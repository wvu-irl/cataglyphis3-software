#ifndef SAMPLE_SEARCH_H
#define SAMPLE_SEARCH_H
#include <ros/ros.h>
#include <computer_vision/SearchForSamples.h>

class SampleSearch
{
public:
	// Methods
	SampleSearch(); // Constructor
	bool searchForSamples(computer_vision::SearchForSamples::Request &req, computer_vision::SearchForSamples::Response &res);
	// Members
	ros::NodeHandle nh;
	ros::ServiceServer searchForSamplesServ;
};

#endif // SAMPLE_SEARCH_H
