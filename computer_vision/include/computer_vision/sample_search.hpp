/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
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
#include <computer_vision/settings.h>

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
	int whiteSampleCounter;
	int silverSampleCounter;
	int blueOrPurpleSampleCounter;
	int pinkSampleCounter;
	int redSampleCounter;
	int orangeSampleCounter;
	int yellowSampleCounter;
};

#endif // SAMPLE_SEARCH_H
