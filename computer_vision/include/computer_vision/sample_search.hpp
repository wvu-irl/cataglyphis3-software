#ifndef SAMPLE_SEARCH_H
#define SAMPLE_SEARCH_H
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <computer_vision/patch.hpp>
#include <computer_vision/capture_class.hpp> 
#include <messages/CVSearchCmd.h>
#include <messages/CVSampleProps.h>
#include <messages/CVSamplesFound.h>
#include <computer_vision/SegmentImage.h> 
#include <computer_vision/ImageProbabilities.h> 
#include <computer_vision/ExtractColor.h> 
#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <computer_vision/sample_types.h>
#include <messages/CVSetSampleParams.h>

class SampleSearch
{
private:
	struct timeval localTimer;
public:
	// Methods
	SampleSearch(); // Constructor
	bool searchForSamples(messages::CVSearchCmd::Request &req, messages::CVSearchCmd::Response &res);
	bool setSampleParams(messages::CVSetSampleParams::Request &req, messages::CVSetSampleParams::Response &res);
	void createFolderForImageData();
	void createFileForImageData();
	void loadCalibrationData();
	void drawResultsOnImage(const std::vector<int> &blobsOfInterest, 
							const std::vector<int> &blobsOfNotInterest, 
							const std::vector<int> &coordinates, 
							const std::vector<int> &types, 
							const std::vector<float> &probabilities);
	void saveLowAndHighProbabilityBlobs(const std::vector<float> &probabilities, 
										const std::vector<int> &coordinates);
	void saveTopROICandidates(const std::vector<int> &blobsOfInterest,
                              const std::vector<int> &blobsOfNotInterest, 
                              const std::vector<int> &types, 
                              const std::vector<float> &probabilities, 
                              const int &roi);
	std::vector<double> calculateFlatGroundPositionOfPixel(int u, int v);
	// Members
	ros::NodeHandle nh;
	ros::Publisher searchForSamplesPub;
	ros::ServiceServer searchForSamplesServ;
	ros::ServiceServer setSetSampleParamsServ;
	ros::ServiceClient segmentImageClient;
	ros::ServiceClient classifierClient;
	ros::ServiceClient extractColorClient;
	computer_vision::SegmentImage segmentImageSrv;
	computer_vision::ImageProbabilities imageProbabilitiesSrv;
	computer_vision::ExtractColor extractColorSrv;
	messages::CVSamplesFound searchForSamplesMsgOut;
	const double G_SENSOR_HEIGHT = 1.5;
	const double G_IMAGE_WIDTH = 5792;
	const double G_IMAGE_HEIGHT = 5792;
	const double G_FOCAL_LENGTH = 0.01;
	const double G_SIZE_OF_PIXEL = 4.1436e-6;
	const double G_IMAGE_SENSOR_PIXEL_WIDTH = 4.1436e-6;
	const double G_IMAGE_SENSOR_PIXEL_HEIGHT = 4.1436e-6;
	cv::Mat G_rotation_camera_2_robot;
	boost::filesystem::path G_data_folder;
	boost::filesystem::path G_data_folder_full_images;
	std::string G_data_folder_name;
	std::string G_blob_image_name;
	int G_image_index;
	std::ofstream G_outputInfoFile;
	std::string G_info_filename;
	struct roi_t
	{
		std::vector<float> probabilities;
		std::vector<SAMPLE_TYPE_T> types;
		std::vector<std::string> paths;
	}; 
	const int MAX_NUMBER_OF_ROIS = 16;
	std::vector<roi_t> _rois;
	std::vector<float> G_cach_probabilities;
	std::vector<float> G_rock_probabilities;
	std::vector<float> G_hard_probabilities;
	std::vector<float> G_sample_probabilities;
	std::vector<float> G_sample_ids;
	const float cachSigCoef[4] = {0.014343043024976,0.996308298545352,0.556984792625824,12.655992533439981};
	const float hardSigCoef[4] = {0.009590048374287,0.996948659156515,0.513078429713242,15.391445601972140};
	const float rockSigCoef[4] = {0.046567867798548,0.996253031121210,0.472406154111716,21.523462524555410};
};

#endif // SAMPLE_SEARCH_H
