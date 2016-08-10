#include <ros/ros.h>
#include <ros/package.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

#ifndef CALIBRATE_CAMERA_HPP
#define CALIBRATE_CAMERA_HPP

class CalibrateCamera
{
private:

public:
	CalibrateCamera();

	cv::Mat _dst;
	cv::Mat _dstCopy;
	cv::Mat _calibrationMask;
	cv::Mat _saveMask;

	int _robotWidthMax = 5792;
	int _robotWidthSlider = 0;
	int _robotHeightMax = 5792;
	int _robotHeightSlider = 0;
	int _robotShiftMax = 5792;
	int _robotShiftSlider = 0;
	int _radiusMax = 4095;
	int _radiusSlider = 4095;
	int _poleWidthMax = 5792;
	int _poleWidthSlider = 0;
	int _poleHeightMax = 5792/2;
	int _poleHeightSlider = 0;

	static void onTrackbarRobotWidth(int, void*){};
	static void onTrackbarRobotHeight(int, void*){};
	static void onTrackbarRobotShift(int, void*){};
	static void onTrackbarRadius(int, void*){};
	static void onTrackbarPoleWidth(int, void*){};
	static void onTrackbarPoleHeight(int, void*){};

	bool _showBody = false;
	bool _showPole = false;
	bool _showGrabber = false;
	bool _showRadius = false;

	static void bodyCheckBox(int state, void* object){
		if(state==true)
		{
			((CalibrateCamera*)object)->_showBody=true;
			ROS_INFO("True.");
		}
		else
		{
			((CalibrateCamera*)object)->_showBody=false;
			ROS_INFO("False.");
		}
	};
	static void poleCheckBox(int state, void* object){
		if(state==true)
		{
			((CalibrateCamera*)object)->_showPole=true;
		}
		else
		{
			((CalibrateCamera*)object)->_showPole=false;
		}
	};
	static void grabberCheckBox(int state, void* object){
		if(state==true)
		{
			((CalibrateCamera*)object)->_showGrabber=true;
		}
		else
		{
			((CalibrateCamera*)object)->_showGrabber=false;
		}
	};
	static void radiusCheckBox(int state, void* object){
		if(state==true)
		{
			((CalibrateCamera*)object)->_showRadius=true;
		}
		else
		{
			((CalibrateCamera*)object)->_showRadius=false;
		}
	};
	static void calibratePoseButton(int state, void* object){
		ROS_WARN("Calibrate pose not implemented yet.");
	}
	static void updateImageButton(int state, void* object){
		((CalibrateCamera*)object)->updateImage();
	};
	static void saveCalibrationButton(int state, void*){
		ROS_WARN("Save not implemented yet.");
	};

	void displayImage();
	void updateImage();
	void loadCalibration();
	void saveCalibration();
};

#endif //CALIBRATE_CAMERA_HPP