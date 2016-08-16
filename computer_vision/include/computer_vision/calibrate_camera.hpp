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