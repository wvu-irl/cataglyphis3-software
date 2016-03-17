#ifndef SAMPLE_SEARCH_H
#define SAMPLE_SEARCH_H
#include <ros/ros.h>
#include <messages/CVSearchCmd.h>
#include <string>
#include <fstream>

class SampleSearch
{
public:
	// Methods
	SampleSearch(); // Constructor
	int loadLookupTable(std::string filename);
	bool searchForSamples(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res);
	// Members
	ros::NodeHandle nh;
	ros::ServiceServer searchForSamplesServ;

	unsigned char ***G_lookup_table;
};

#endif // SAMPLE_SEARCH_H
