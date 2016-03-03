#include <ros/ros.h>
#include <computer_vision/sample_search.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "computer_vision_node");
	ROS_INFO("computer_vision_node running...");
	SampleSearch samplesSearch;
	ros::spin();
	return 0;
}