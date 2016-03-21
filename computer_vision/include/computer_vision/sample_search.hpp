#ifndef SAMPLE_SEARCH_H
#define SAMPLE_SEARCH_H
#include <ros/ros.h>
#include <computer_vision/capture_class.hpp> 
#include <messages/CVSearchCmd.h>
#include <string>
#include <fstream>

class SampleSearch
{
public:
	// Methods
	SampleSearch(); // Constructor
	//void initialize_camera();
	//int check_camera();
	//void display_image();
	//int loadLookupTable(std::string filename);
	//bool searchForSamples(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res);
	// Members
	// ros::NodeHandle nh;
	// ros::ServiceServer searchForSamplesServ;

	//Capture capture;
	//segmentation class
	//recognition by color class
	//recognition by deep learning class

	// unsigned char ***G_lookup_table;
};

#endif // SAMPLE_SEARCH_H
