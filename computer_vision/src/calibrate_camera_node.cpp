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

#include <computer_vision/capture_class.hpp>
#include <computer_vision/calibrate_camera.hpp>
#include <boost/filesystem.hpp>
#include <computer_vision/patch.hpp>

int main(int argc, char **argv)
{
	//Initialize ROS Node
	ros::init(argc, argv, "calibrate_camera_node");
	ros::Time::init();
	ROS_INFO("calibrate_camera_node running...");

	//Get Path for Saving Images
	boost::filesystem::path P( ros::package::getPath("computer_vision") );

	//Initialize Camera and Caliration Objects
	Capture capture;
	CalibrateCamera calibrateCamera;

	//Capture Initial Image
	if(argc>1)
	{
		boost::filesystem::path P( ros::package::getPath("computer_vision") );
		std::string filename = P.string() + "/f10.jpg";
		cv::Mat tempImage = cv::imread(filename.c_str());
		calibrateCamera.setImage(tempImage);
	}
	else
	{
		if(capture.capture_image()==0)
		{
			ROS_ERROR("Could not capture initial image from camera. Terminating...");
			return 0;
		}
		else
		{
			cv::Mat tempImage = capture.image_Mat.clone();
			calibrateCamera.setImage(tempImage);
		}
	}

	//Main Loop
	calibrateCamera.loadCalibration();
	calibrateCamera.initializeTrackbars();
	calibrateCamera.updateImage();
	while(ros::ok())
	{
		calibrateCamera.displayImage();

		char keyPress = cv::waitKey(30);
		if(calibrateCamera._captureImage == true || keyPress == 'r')
		{
			calibrateCamera._captureImage = false;
			if(capture.capture_image()==0)
			{
				ROS_ERROR("Could not capture a new image from camera. Terminating...");
			}
			else
			{
				cv::Mat tempImg = cv::Mat(capture.image_Mat.clone());
				std::string picture_name = P.string() + "/data/keyboard_images/" + patch::currentDateTime() + ".jpg";
				imwrite(picture_name.c_str(), tempImg);
				calibrateCamera.setImage(tempImg);
				calibrateCamera.updateImage();
			}
		}
		else if(keyPress == 'p')
		{
			if(capture.capture_image()==0)
			{
				ROS_ERROR("Could not capture a new image from camera. Terminating...");
			}
			else
			{
				cv::Mat tempImg = cv::Mat(capture.image_Mat.clone());
				std::string picture_name = P.string() + "/data/keyboard_images/" + patch::currentDateTime() + ".jpg";
				imwrite(picture_name.c_str(), tempImg);

			}
		}
		else if(calibrateCamera._exit == true || keyPress == 'q')
		{
			break;
		}

		ros::spinOnce();
	}

	return 0;
}
