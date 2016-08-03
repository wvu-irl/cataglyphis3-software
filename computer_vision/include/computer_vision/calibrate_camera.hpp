#include <ros/ros.h>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#ifndef CALIBRATE_CAMERA_HPP
#define CALIBRATE_CAMERA_HPP

class CalibrateCamera
{
private:
	cv::Mat _inputImage;
public:
	CalibrateCamera();
	bool refreshImage(cv::Mat src);
	void onTrackbar( int, void* );

	/// Global Variables
	const int alpha_slider_max = 100;
	int alpha_slider;
	double alpha;
	double beta;

	/// Matrices to store images
	cv::Mat dst;
};

#endif //CALIBRATE_CAMERA_HPP
