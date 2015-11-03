#include <ros/ros.h>
#include "robot_control/exec.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "exec_node");
	Exec exec;
	ros::Rate loopRate(exec.loopRate);
	ros::spinOnce();
	while(ros::ok())
	{
		exec.run();
		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}
