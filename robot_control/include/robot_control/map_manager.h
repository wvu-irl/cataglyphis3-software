#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H
#include <ros/ros.h>
#include <robot_control/Waypoint.h>
#include <robot_control/RegionsOfInterest.h>
#include <robot_control/ModifyROI.h>
#include <messages/ROIGridMap.h>
#include <messages/KeyframeList.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <vector>

class MapManager
{
public:
	// Methods
	MapManager(); // Constructor
	bool listROI(robot_control::RegionsOfInterest::Request &req, robot_control::RegionsOfInterest::Response &res);
	bool modROI(robot_control::ModifyROI::Request &req, robot_control::ModifyROI::Response &res);
	bool mapROI(messages::ROIGridMap::Request &req, messages::ROIGridMap::Response &res);
	void keyframesCallback(const messages::KeyframeList::ConstPtr& msg);
	void updateCurrentROI();
	// Members
	ros::NodeHandle nh;
	ros::ServiceServer roiServ;
	ros::ServiceServer modROIServ;
	ros::ServiceServer mapROIServ;
	ros::Subscriber keyframesSub;
	robot_control::Waypoint waypoint;
	robot_control::ROI ROI;
	std::vector<robot_control::ROI> regionsOfInterest;
	grid_map::GridMap satMap;
	messages::KeyframeList keyframes;
	ros::Publisher satMapPub;
	grid_map_msgs::GridMap satMapMsg;
	int currentROI;

};

#endif // MAP_MANAGER_H
