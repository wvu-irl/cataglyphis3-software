#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H
#include <ros/ros.h>
#include <robot_control/Waypoint.h>
#include <robot_control/RegionsOfInterest.h>
#include <robot_control/ModifyROI.h>
#include <robot_control/CurrentROI.h>
#include <robot_control/SearchMap.h>
#include <messages/ROIGridMap.h>
#include <messages/KeyframeList.h>
#include <messages/RobotPose.h>
#include <messages/SLAMPoseOut.h>
#include <messages/CreateROIKeyframe.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "map_layers.h"
#include <vector>
#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI

class MapManager
{
public:
	// Methods
	MapManager(); // Constructor
	bool listROI(robot_control::RegionsOfInterest::Request &req, robot_control::RegionsOfInterest::Response &res);
	bool modROI(robot_control::ModifyROI::Request &req, robot_control::ModifyROI::Response &res);
	bool mapROI(messages::ROIGridMap::Request &req, messages::ROIGridMap::Response &res);
	bool searchMapCallback(robot_control::SearchMap::Request &req, robot_control::SearchMap::Response &res);
	void keyframesCallback(const messages::KeyframeList::ConstPtr& msg);
	void globalPoseCallback(const messages::RobotPose::ConstPtr& msg);
	void keyframeRelPoseCallback(const messages::SLAMPoseOut::ConstPtr& msg);
	void resetGlobalMapLayers(int startIndex, int endIndex);
	void gridMapAddLayers(int layerStartIndex, int layerEndIndex ,grid_map::GridMap& map);
	// Members
	ros::NodeHandle nh;
	ros::ServiceServer roiServ;
	ros::ServiceServer modROIServ;
	ros::ServiceServer mapROIServ;
	ros::ServiceServer searchMapServ;
	ros::ServiceClient createROIKeyframeClient;
	ros::Subscriber keyframesSub;
	ros::Subscriber globalPoseSub;
	ros::Subscriber keyframeRelPoseSub;
	ros::Publisher currentROIPub;
	robot_control::Waypoint waypoint;
	robot_control::ROI ROI;
	std::vector<robot_control::ROI> regionsOfInterest;
	grid_map::GridMap globalMap;
	grid_map::GridMap searchLocalMap;
	int searchLocalMapROINum;
	ros::Publisher globalMapPub;
	grid_map_msgs::GridMap globalMapMsg;
	messages::RobotPose globalPose;
	messages::SLAMPoseOut keyframeRelPose;
	messages::KeyframeList keyframes;
	messages::CreateROIKeyframe createROIKeyframeSrv;
	grid_map::GridMap currentKeyframe;
	float currentCellValue;
	float possibleNewCellValue;
	unsigned int keyframeCallbackSerialNum;
	grid_map::GridMap ROIKeyframe;
	float sigmaROIX;
	float sigmaROIY;
	const float numSigmasROIAxis = 3.0;
	float ROIX;
	float ROIY;
	float searchLocalMapToROIAngle;
	float keyframeOriginalXLen; // ***
	float keyframeOriginalYLen; // ***
	float keyframeXPos;
	float keyframeYPos;
	grid_map::Position keyframeCoord;
	grid_map::Position searchLocalMapCoord;
	//float keyframeOriginalYCoord;
	float keyframeTransformXLen; // ***
	float keyframeTransformYLen; // ***
	float keyframeHeading;
	grid_map::Position globalTransformCoord;
	//float globalYTransformPos;
	robot_control::CurrentROI currentROIMsg;
	const float mapResolution = 1.0; // m
	const float searchLocalMapLength = 40.0; // m
	const float searchLocalMapWidth = 40.0; // m
	const int sampleProbPeak = 1000;
};

#endif // MAP_MANAGER_H
