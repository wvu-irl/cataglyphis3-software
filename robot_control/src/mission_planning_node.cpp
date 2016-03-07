#include <ros/ros.h>
#include "robot_control/mission_planning.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mission_planning_node");
	MissionPlanning missionPlanning;
	ros::Rate loopRate(missionPlanning.loopRate);
	ros::Duration(2).sleep();
	ros::spinOnce();
    //missionPlanning.run();
    //ros::spinOnce();
    while(ros::ok())
	{
		missionPlanning.run();
		ros::spinOnce();
		loopRate.sleep();
    }
	return 0;
}
