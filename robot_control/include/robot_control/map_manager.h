#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H
#include <ros/ros.h>
#include <robot_control/Waypoint.h>
#include <robot_control/WaypointsOfInterest.h>
#include <robot_control/RegionsOfInterest.h>
#include <robot_control/ModifyROI.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <vector>

class MapManager
{
public:
	// Methods
	MapManager(); // Constructor
	bool WOI(robot_control::WaypointsOfInterest::Request &req, robot_control::WaypointsOfInterest::Response &res);
	bool ROI(robot_control::RegionsOfInterest::Request &req, robot_control::RegionsOfInterest::Response &res);
	bool modROI(robot_control::ModifyROI::Request &req, robot_control::ModifyROI::Response &res);
	// Members
	ros::NodeHandle nh;
	ros::ServiceServer woiServ;
	ros::ServiceServer roiServ;
	ros::ServiceServer modROIServ;
	std::vector <robot_control::Waypoint> waypointsOfInterestVector;
	robot_control::Waypoint waypoint;
	std::vector<robot_control::Waypoint> regionsOfInterest;
	grid_map::GridMap satMap;
	ros::Publisher satMapPub;
	grid_map_msgs::GridMap satMapMsg;
};

#endif // MAP_MANAGER_H
