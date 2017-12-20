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
#ifndef SEGMENTATION_H
#define SEGMENTATION_H
#include <ros/ros.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <computer_vision/capture_class.hpp>
#include <computer_vision/SegmentImage.h>
#include <computer_vision/ExtractColor.h>
#include <string>
#include <fstream>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <computer_vision/patch.hpp>
#include <computer_vision/sample_types.h>
#include <computer_vision/settings.h>

class Segmentation
{
public:
	// Methods
	Segmentation(); // Constructor
	bool segmentImage(computer_vision::SegmentImage::Request &req, computer_vision::SegmentImage::Response &res);
	bool extractColor(computer_vision::ExtractColor::Request &req, computer_vision::ExtractColor::Response &res);
	int loadLookupTable(std::string filename); //30MB of memory for 255x255x255 lookup table
	int setCalibration();
	void assign_colors(cv::Mat& I, uchar*** &lut);
	std::vector<cv::Rect> getIndividualBlobs(const cv::Mat& segmented);
	std::vector<int> writeSegmentsToFile(std::vector<cv::Rect> rectangles, const cv::Mat& origional);
	// Members
	ros::NodeHandle nh;
	ros::ServiceServer segmentationServ;
	ros::ServiceServer extractColorServ;
	Capture capture;
	unsigned char ***G_lookup_table;
	cv::Mat calibrationMask;
};

#endif // SEGMENTATION_H
