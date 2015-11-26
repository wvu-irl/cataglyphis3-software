#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H
#include <ros/ros.h>
#include <robot_control/Waypoint.h>
#include <robot_control/WaypointsOfInterest.h>
#include <vector>

class MapManager
{
public:
	// Methods
	MapManager(); // Constructor
	bool WOI(robot_control::WaypointsOfInterest::Request &req, robot_control::WaypointsOfInterest::Response &res);
	// Members
	ros::NodeHandle nh;
	ros::ServiceServer woiServ;
	std::vector <robot_control::Waypoint> waypointsOfInterestVector;
	robot_control::Waypoint waypoint;
};

#endif // MAP_MANAGER_H
