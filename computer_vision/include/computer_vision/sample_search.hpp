#ifndef SAMPLE_SEARCH_H
#define SAMPLE_SEARCH_H
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <computer_vision/capture_class.hpp> 
#include <messages/CVSearchCmd.h>
#include <messages/CVSamplesFound.h>
#include <computer_vision/SegmentImage.h> 
#include <string>
#include <fstream>
#include <vector>

class SampleSearch
{
public:
	// Methods
	SampleSearch(); // Constructor
	bool searchForSamples(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res);
	int drawResultsOnImage(const std::vector<int> &binary, const std::vector<int> &u, const std::vector<int> &v, cv::Mat &I);
	// Members
	ros::NodeHandle nh;
	ros::Publisher searchForSamplesPub;
	ros::ServiceServer searchForSamplesServ;
	ros::ServiceClient segmentImageClient;
	computer_vision::SegmentImage segmentImageSrv;
	messages::CVSamplesFound searchForSamplesMsgOut;
};

#endif // SAMPLE_SEARCH_H
