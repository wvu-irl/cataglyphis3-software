#include <ros/ros.h>
#include <robot_control/map_manager.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_manager_node");
	MapManager mapManager;
	ros::spin();
	return 0;
}
