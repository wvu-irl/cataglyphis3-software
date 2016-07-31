#include <ros/ros.h>
#include <robot_control/map_manager.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_manager_node");
	MapManager mapManager;
    ROS_INFO("Map Manager Running...");
	ros::spin();
	return 0;
}
