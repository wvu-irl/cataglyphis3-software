#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H
#include <ros/ros.h>
#include <robot_control/Waypoint.h>
#include <robot_control/RegionsOfInterest.h>
#include <robot_control/ModifyROI.h>
#include <robot_control/CurrentROI.h>
#include <robot_control/SearchMap.h>
#include <messages/KeyframeList.h>
#include <messages/RobotPose.h>
#include <messages/SLAMPoseOut.h>
#include <messages/CreateROIKeyframe.h>
#include <messages/CVSamplesFound.h>
#include <messages/GlobalMapPathHazards.h>
#include <messages/SearchLocalMapInfo.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "map_layers.h"
#include <vector>
#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI

#define EVANSDALE
//#define UHS
//#define INSTITUTE_PARK

#ifdef EVANSDALE
#include "evansdale_maps.h"
#endif // EVANSDALE

#ifdef UHS
#include "uhs_maps.h"
#endif // UHS

#ifdef INSTITUTE_PARK
#include "institute_park_maps.h"
#endif // INSTITUTE_PARK

class MapManager
{
public:
	// Methods
	MapManager(); // Constructor
	bool listROI(robot_control::RegionsOfInterest::Request &req, robot_control::RegionsOfInterest::Response &res);
	bool modROI(robot_control::ModifyROI::Request &req, robot_control::ModifyROI::Response &res);
	bool searchMapCallback(robot_control::SearchMap::Request &req, robot_control::SearchMap::Response &res);
	bool globalMapPathHazardsCallback(messages::GlobalMapPathHazards::Request &req, messages::GlobalMapPathHazards::Response &res);
	bool searchLocalMapInfoCallback(messages::SearchLocalMapInfo::Request &req, messages::SearchLocalMapInfo::Response &res);
	void keyframesCallback(const messages::KeyframeList::ConstPtr& msg);
	void globalPoseCallback(const messages::RobotPose::ConstPtr& msg);
	void keyframeRelPoseCallback(const messages::SLAMPoseOut::ConstPtr& msg);
	void cvSamplesFoundCallback(const messages::CVSamplesFound::ConstPtr& msg);
	void gridMapResetLayers(int startIndex, int endIndex, grid_map::GridMap& map);
	void gridMapAddLayers(int layerStartIndex, int layerEndIndex, grid_map::GridMap& map);
	void donutSmash(int layerStartIndex, int layerEndIndex, grid_map::GridMap& map, grid_map::Position pos);
	void addFoundSamples(int layerStartIndex, int layerEndIndex, grid_map::GridMap& map, grid_map::Position pos, float heading);
	void rotateCoord(float origX, float origY, float &newX, float &newY, float angleDeg);
	void rotateCoord(double origX, double origY, double &newX, double &newY, double angleDeg);
	void writeSatMapIntoGlobalMap();
	void writeKeyframesIntoGlobalMap();
	void northTransformROIs();
	void updateNorthTransformedMapData();
	void smoothDriveabilityLayer();
	//void smoothNumericLayer();
	// Members
	ros::NodeHandle nh;
	ros::ServiceServer roiServ;
	ros::ServiceServer modROIServ;
	ros::ServiceServer searchMapServ;
	ros::ServiceServer globalMapPathHazardsServ;
	ros::ServiceServer searchLocalMapInfoServ;
	ros::ServiceClient createROIKeyframeClient;
	ros::Subscriber keyframesSub;
	ros::Subscriber globalPoseSub;
	ros::Subscriber keyframeRelPoseSub;
	ros::Subscriber cvSamplesFoundSub;
	ros::Publisher currentROIPub;
	ros::Publisher globalMapPub;
	ros::Publisher searchLocalMapPub;
	grid_map_msgs::GridMap globalMapMsg;
	grid_map_msgs::GridMap searchLocalMapMsg;
	robot_control::Waypoint waypoint;
	robot_control::ROI ROI;
	std::vector<robot_control::ROI> regionsOfInterest;
	grid_map::GridMap globalMap;
	grid_map::Length globalMapSize;
	grid_map::Length satMapSize;
	grid_map::Size globalMapRowsCols;
	grid_map::GridMap globalMapTemp;
	grid_map::GridMap searchLocalMap;
	int searchLocalMapROINum;
	bool searchLocalMapExists;
	grid_map::Polygon globalMapPathHazardsPolygon;
	std::vector<grid_map::Position> globalMapPathHazardsVertices;
	float globalMapPathHazardsPolygonHeading;
	float globalMapPathHazardValue;
	grid_map::Position globalMapPathHazardPosition;
	messages::RobotPose globalPose;
	float previousNorthAngle; // deg
	messages::SLAMPoseOut keyframeRelPose;
	messages::KeyframeList keyframes;
	messages::CreateROIKeyframe createROIKeyframeSrv;
	grid_map::GridMap currentKeyframe;
	float currentCellValue;
	float possibleNewCellValue;
	unsigned int keyframeWriteIntoGlobalMapSerialNum;
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
	grid_map::Position satMapToGlobalMapPos;
	//float globalYTransformPos;
	robot_control::CurrentROI currentROIMsg;
	messages::CVSamplesFound cvSamplesFoundMsg;
	grid_map::Position globalMapOrigin;
	const float mapResolution = 1.0; // m
	const float searchLocalMapLength = 40.0; // m
	const float searchLocalMapWidth = 40.0; // m
	const float sampleProbPeak = 1.0;
	const int smoothDriveabilityNumNeighborsToChangeValue = 6;
};

#endif // MAP_MANAGER_H
