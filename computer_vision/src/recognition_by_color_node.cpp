#include <ros/ros.h>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "recognition_by_color_node");
	ROS_INFO("recognition_by_color_node running...");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}