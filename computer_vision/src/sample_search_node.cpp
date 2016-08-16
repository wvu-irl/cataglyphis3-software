#include <ros/ros.h>
#include <computer_vision/sample_search.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sample_search_node");
	ROS_INFO("sample_search_node running...");
	SampleSearch samplesSearch;
	samplesSearch.createFolderForImageData();
	samplesSearch.createFileForImageData();
	samplesSearch.loadCalibrationData();
	//samplesSearch.calculateFlatGroundPositionOfPixel(5792/4,5792/4);
	ros::spin();
	return 0;
} 
