#include <ros/ros.h>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "safe_pathing_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	ROS_INFO("safe_pathing_node running...");

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
