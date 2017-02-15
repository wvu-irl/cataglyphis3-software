#include <ros/ros.h>
#include "robot_control/exec.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "exec_node");
	Exec exec;
	ros::Rate loopRate(exec.loopRate);
	ros::spinOnce();
    double prevTime = ros::Time::now().toSec();
    double deltaTime;
    double currentTime;
	while(ros::ok())
	{
		exec.run();
		ros::spinOnce();
		loopRate.sleep();
        currentTime = ros::Time::now().toSec();
        deltaTime = currentTime - prevTime;
        //ROS_INFO_THROTTLE(1,"deltaTime = %f",deltaTime);
        prevTime = currentTime;
	}
	return 0;
}
