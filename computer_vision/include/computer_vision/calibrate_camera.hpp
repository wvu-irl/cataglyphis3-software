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
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <boost/filesystem.hpp>
#include <fstream>
#include <string>

#ifndef CALIBRATE_CAMERA_HPP
#define CALIBRATE_CAMERA_HPP

class CalibrateCamera
{
public:
	CalibrateCamera();
	void setImage(cv::Mat &src);
	void initializeTrackbars();
	void displayImage();
	void updateImage();
	void loadCalibration();
	void saveCalibration();

	bool _captureImage = false;
	bool _exit = false;

private:
	cv::Mat _dst;
	cv::Mat _dstCopy;
	cv::Mat _calibrationMask;
	cv::Mat _saveMask;
	cv::Mat _cameraMatrix;
	cv::Mat _distortionCoefficients;

	static const int _LINE_THICKNESS = 16;
	static const int _LINE_TYPE = 8;
	static const int _LINE_SHIFT = 0;

	int _robotWidthMax = 5792;
	int _robotWidthSlider = 0;
	int _robotHeightMax = 5792;
	int _robotHeightSlider = 0;
	int _robotShiftMax = 5792;
	int _robotShiftSlider = 0;
	int _radiusMax = 4095;
	int _radiusSlider = 0;
	int _poleWidthMax = 5792;
	int _poleWidthSlider = 0;
	int _poleHeightMax = 5792/2;
	int _poleHeightSlider = 0;
	int _grabberWidthMax = 5792;
	int _grabberWidthSlider = 0;
	int _grabberHeightMax = 5792/2;
	int _grabberHeightSlider = 0;
	int _grabberShiftMax = 5792;
	int _grabberShiftSlider = 0;
	int _rollMax = 100;
	int _rollSlider = 0;//_rollMax/2;
	int _rollOffset = -_rollMax/2;
	int _pitchMax = 250;
	int _pitchSlider = 0;//_pitchMax/2;
	int _pitchOffset = -_pitchMax/2;
	int _yawMax = 250;
	int _yawSlider = 0;//_yawMax/2;
	int _yawOffset = -_yawMax/2;

	bool _showBody = false;
	bool _showPole = false;
	bool _showGrabber = false;
	bool _showRadius = false;
	bool _showLines = false;

	const double G_FOCAL_LENGTH = 0.01;
	const double IMAGE_SENSOR_PIXEL_SIZE = 4.1436e-6;
	const double G_SENSOR_HEIGHT = 1.45;

	double _rollAngle = 0;
	double _pitchAngle = 0;
	double _yawAngle = 0;

	//interface callbacks
	static void onTrackbarRobotWidth(int, void*)
	{

	};

	static void onTrackbarRobotHeight(int, void*)
	{

	};

	static void onTrackbarRobotShift(int, void*)
	{

	};

	static void onTrackbarRadius(int, void*)
	{

	};

	static void onTrackbarPoleWidth(int, void*)
	{

	};

	static void onTrackbarPoleHeight(int, void*)
	{

	};

	static void onTrackbarGrabberWidth(int, void*)
	{

	};

	static void onTrackbarGrabberHeight(int, void*)
	{

	};

	static void onTrackbarGrabberShift(int, void*)
	{

	};

	static void onTrackbarRoll(int, void*)
	{

	};

	static void onTrackbarPitch(int, void*)
	{

	};

	static void onTrackbarYaw(int, void*)
	{

	};

	static void bodyCheckBox(int state, void* object)
	{
		((CalibrateCamera*)object)->bodyCheckBoxImplementation(state);
	};

	static void poleCheckBox(int state, void* object)
	{
		((CalibrateCamera*)object)->poleCheckBoxImplementation(state);
	};

	static void grabberCheckBox(int state, void* object)
	{
		((CalibrateCamera*)object)->grabberCheckBoxImplementation(state);
	};

	static void radiusCheckBox(int state, void* object)
	{
		((CalibrateCamera*)object)->radiusCheckBoxImplementation(state);
	};

	static void linesCheckBox(int state, void* object)
	{
		((CalibrateCamera*)object)->linesCheckBoxImplementation(state);
	};

	static void captureImageButton(int state, void* object)
	{
		((CalibrateCamera*)object)->captureImageButtonImplementation();
	};

	static void updateImageButton(int state, void* object)
	{
		((CalibrateCamera*)object)->updateImageButtonImplementation();
	};

	static void saveCalibrationButton(int state, void* object)
	{
		 ((CalibrateCamera*)object)->saveCalibrationButtonImplementation();
	};

	static void loadCalibrationButton(int state, void* object)
	{
		((CalibrateCamera*)object)->loadCalibrationButtonImplementation();
	};

	static void exitButton(int state, void* object)
	{
		((CalibrateCamera*)object)->exitButtonImplementation();
	};

	//interface implementations
	void bodyCheckBoxImplementation(int state);
	void poleCheckBoxImplementation(int state);
	void grabberCheckBoxImplementation(int state);
	void radiusCheckBoxImplementation(int state);
	void linesCheckBoxImplementation(int state);
	void captureImageButtonImplementation();
	void updateImageButtonImplementation();
	void saveCalibrationButtonImplementation();
	void loadCalibrationButtonImplementation();
	void exitButtonImplementation();
};

#endif //CALIBRATE_CAMERA_HPP
