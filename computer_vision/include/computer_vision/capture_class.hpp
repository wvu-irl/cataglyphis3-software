#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <opencv2/opencv.hpp>
#include <gphoto2/gphoto2-camera.h>
#include <gphoto2/gphoto2.h>
#include <std_msgs/UInt8MultiArray.h>

#ifndef CAPTURE_CLASS_HPP
#define CAPTURE_CLASS_HPP

class Capture
{
private:
	Camera *camera;  
	GPContext *context;
	int retval;
	int camera_detection_error;
	std_msgs::UInt8MultiArray image_UInt8MultiArray;

	int initialize_camera();
	int auto_detect_camera();
	int capture_image_to_buffer();
	int decode_image_from_buffer_to_mat();
	//void write_buffer_to_file(string filename);
	int capture_success();
	void close_camera();

	//void error_func (GPContext *context, const char *format, va_list args, void *data) ;
	//void message_func (GPContext *context, const char *format, va_list args, void *data);
	//static void capture_to_file(Camera *camera, GPContext *context, char *fn);
	static int capture_to_memory(Camera *camera, GPContext *context, const char **ptr, unsigned long int *size);
public:
	cv::Mat image_Mat;
	int capture_image();
};

#endif //CAPTURE_CLASS_HPP
