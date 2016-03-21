#ifndef SEGMENTATION_H
#define SEGMENTATION_H
#include <ros/ros.h>
#include <computer_vision/capture_class.hpp> 
#include <computer_vision/SegmentImage.h> 
#include <string>
#include <fstream>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <computer_vision/patch.hpp>

class Segmentation: public Capture
{
public:
	// Methods
	Segmentation(); // Constructor
	bool segmentImage(computer_vision::SegmentImage::Request &req, computer_vision::SegmentImage::Response &res);
	int loadLookupTable(std::string filename); //30MB of memory for 255x255x255 lookup table
	void assign_colors(cv::Mat& I, uchar*** &lut);
	std::vector<cv::Rect> getIndividualBlobs(const cv::Mat& segmented);
	int writeSegmentsToFile(std::vector<cv::Rect> rectangles, const cv::Mat& origional);
	// Members
	ros::NodeHandle nh;
	ros::ServiceServer segmentationServ;
	unsigned char ***G_lookup_table;
};

#endif // SEGMENTATION_H