#ifndef SAMPLE_SEARCH_H
#define SAMPLE_SEARCH_H
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <computer_vision/capture_class.hpp> 
#include <messages/CVSearchCmd.h>
#include <messages/CVSampleProps.h>
#include <messages/CVSamplesFound.h>
#include <computer_vision/SegmentImage.h> 
#include <computer_vision/ImageProbabilities.h> 
#include <string>
#include <fstream>
#include <vector>

class SampleSearch
{
public:
	// Methods
	SampleSearch(); // Constructor
	bool searchForSamples(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res);
	void drawResultsOnImage(const std::vector<int> &binary, const std::vector<int> &coordinates);
	std::vector<double> calculateFlatGroundPositionOfPixel(int u, int v);
	// Members
	ros::NodeHandle nh;
	ros::Publisher searchForSamplesPub;
	ros::ServiceServer searchForSamplesServ;
	ros::ServiceClient segmentImageClient;
	ros::ServiceClient classifierClient;
	computer_vision::SegmentImage segmentImageSrv;
	computer_vision::ImageProbabilities imageProbabilitiesSrv;
	messages::CVSamplesFound searchForSamplesMsgOut;
	const double G_SENSOR_HEIGHT = 1.385;
	const double G_IMAGE_WIDTH = 5792;
	const double G_IMAGE_HEIGHT = 5792;
	const double G_FOCAL_LENGTH = 0.008;
	const double G_IMAGE_SENSOR_WIDTH = 0.0020347;//4.14e-6 meters square;
	const double G_IMAGE_SENSOR_HEIGHT = 0.0020347;
	const double G_PIXEL2DISTANCE = G_IMAGE_SENSOR_WIDTH/G_IMAGE_WIDTH;
};

#endif // SAMPLE_SEARCH_H
