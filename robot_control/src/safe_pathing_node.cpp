#include <ros/ros.h>
#include <iostream>
#include <robot_control/safe_pathing.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "safe_pathing_node");
	ros::NodeHandle nh;
    ros::Rate loop_rate(600);
	SafePathing safePathing;

	ROS_INFO("safe_pathing_node running...");

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
