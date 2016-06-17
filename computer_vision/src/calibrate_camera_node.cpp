#include <ros/ros.h>
#include <computer_vision/capture_class.hpp> 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibrate_camera_node");
	ROS_INFO("calibrate_camera_node running...");
	Capture capture;
	ros::spin();
	return 0;
} 
