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

	bool selectingPoint1 = false;
	bool selectingPoint2 = false;
	bool selectingPoint3 = false;
	bool selectingPoint4 = false;
	bool startSelectingPoints = false;

	std::vector<cv::Point2f> imageCoordinates;
	std::vector<cv::Point3f> globalCoordinates;

	static void bodyCheckBox(int state, void* object)
	{
		if(state==true)
		{
			((CalibrateCamera*)object)->_showBody=true;
		}
		else
		{
			((CalibrateCamera*)object)->_showBody=false;
		}
	};

	static void poleCheckBox(int state, void* object)
	{
		if(state==true)
		{
			((CalibrateCamera*)object)->_showPole=true;
		}
		else
		{
			((CalibrateCamera*)object)->_showPole=false;
		}
	};

	static void grabberCheckBox(int state, void* object)
	{
		if(state==true)
		{
			((CalibrateCamera*)object)->_showGrabber=true;
		}
		else
		{
			((CalibrateCamera*)object)->_showGrabber=false;
		}
	};

	static void radiusCheckBox(int state, void* object)
	{
		if(state==true)
		{
			((CalibrateCamera*)object)->_showRadius=true;
		}
		else
		{
			((CalibrateCamera*)object)->_showRadius=false;
		}
	};

	static void onMouse(int event, int x, int y, int flags, void* object)
	{
		if(event != CV_EVENT_LBUTTONDOWN)
		{
			return;
		}


		if(((CalibrateCamera*)object)->selectingPoint1==true)
		{
			ROS_INFO("Select point 2");
			((CalibrateCamera*)object)->imageCoordinates.clear();
			((CalibrateCamera*)object)->imageCoordinates.push_back(cv::Point2f(x,y));
			((CalibrateCamera*)object)->selectingPoint1=false;
			((CalibrateCamera*)object)->selectingPoint2=true;
		}
		else if(((CalibrateCamera*)object)->selectingPoint2==true)
		{
			ROS_INFO("Select point 3");
			((CalibrateCamera*)object)->imageCoordinates.push_back(cv::Point2f(x,y));
			((CalibrateCamera*)object)->selectingPoint2=false;
			((CalibrateCamera*)object)->selectingPoint3=true;
		}
		else if(((CalibrateCamera*)object)->selectingPoint3==true)
		{
			ROS_INFO("Select point 4");
			((CalibrateCamera*)object)->imageCoordinates.push_back(cv::Point2f(x,y));
			((CalibrateCamera*)object)->selectingPoint3=false;
			((CalibrateCamera*)object)->selectingPoint4=true;
		}
		else if(((CalibrateCamera*)object)->selectingPoint4==true)
		{
			((CalibrateCamera*)object)->imageCoordinates.push_back(cv::Point2f(x,y));
			((CalibrateCamera*)object)->selectingPoint4=false;
			((CalibrateCamera*)object)->startSelectingPoints=false;
			ROS_INFO("x1=%i,y1=%i",(int)((CalibrateCamera*)object)->imageCoordinates[0].x,(int)((CalibrateCamera*)object)->imageCoordinates[0].y);
			ROS_INFO("x2=%i,y2=%i",(int)((CalibrateCamera*)object)->imageCoordinates[1].x,(int)((CalibrateCamera*)object)->imageCoordinates[1].y);
			ROS_INFO("x3=%i,y3=%i",(int)((CalibrateCamera*)object)->imageCoordinates[2].x,(int)((CalibrateCamera*)object)->imageCoordinates[2].y);
			ROS_INFO("x4=%i,y4=%i",(int)((CalibrateCamera*)object)->imageCoordinates[3].x,(int)((CalibrateCamera*)object)->imageCoordinates[3].y);
		}
		else
		{
			ROS_WARN("No action being executed...");
		}

		//ROS_INFO("x = %i, y = %i", x, y);
	}

	static void calibratePoseButton(int state, void* object)
	{
			ROS_INFO("Select point 1.");
			((CalibrateCamera*)object)->selectingPoint1 = true;
			((CalibrateCamera*)object)->selectingPoint2 = false;
			((CalibrateCamera*)object)->selectingPoint3 = false;
			((CalibrateCamera*)object)->selectingPoint4 = false;
	}
	static void updateImageButton(int state, void* object)
	{
		((CalibrateCamera*)object)->updateImage();
	};
	static void saveCalibrationButton(int state, void*)
	{
		ROS_WARN("Save not implemented yet.");
	};

	void displayImage();
	void updateImage();
	void loadCalibration();
	void saveCalibration();
};

#endif //CALIBRATE_CAMERA_HPP