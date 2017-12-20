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

	//void error_func (GPContext *context, const char *format, va_list args, void *data) ;
	//void message_func (GPContext *context, const char *format, va_list args, void *data);
	//static void capture_to_file(Camera *camera, GPContext *context, char *fn);
	static int capture_to_memory(Camera *camera, GPContext *context, const char **ptr, unsigned long int *size);
public:
	cv::Mat image_Mat;
	std_msgs::UInt8MultiArray image_UInt8MultiArray;

	Capture();
	int initialize_camera();
	int auto_detect_camera();
	int capture_image_to_buffer();
	int decode_image_from_buffer_to_mat();
	//void write_buffer_to_file(string filename);
	int capture_success();
	void close_camera();
	int capture_image();
};

#endif //CAPTURE_CLASS_HPP
