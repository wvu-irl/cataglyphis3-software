#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H
#include <ros/ros.h>
#include <robot_control/Waypoint.h>
#include <robot_control/RegionsOfInterest.h>
#include <robot_control/ModifyROI.h>
#include <robot_control/CurrentROI.h>
#include <robot_control/SearchMap.h>
#include <robot_control/RandomSearchWaypoints.h>
#include <messages/KeyframeList.h>
#include <messages/RobotPose.h>
#include <messages/SLAMPoseOut.h>
#include <messages/CreateROIHazardMap.h>
#include <messages/CVSamplesFound.h>
#include <messages/MapPathHazards.h>
#include <messages/SearchLocalMapInfo.h>
#include <messages/GlobalMapFull.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "map_layers.h"
#include <vector>
#include <time.h>
#include <armadillo>
#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI

#define EVANSDALE
//#define WPI
//#define QUAD
//#define CHESTNUT_RIDGE

#ifdef EVANSDALE
#include "evansdale_map.h"
#endif // EVANSDALE

#ifdef WPI
#include "wpi_map.h"
#endif // WPI

#ifdef QUAD
#include "quad_map.h"
#endif // QUAD

#ifdef CHESTNUT_RIDGE
#include "chestnut_ridge_map.h"
#endif // CHESTNUT_RIDGE

//#include other maps...

class MapManager
{
public:
	// Methods
	MapManager(); // Constructor
	bool listROI(robot_control::RegionsOfInterest::Request &req, robot_control::RegionsOfInterest::Response &res);
	bool modROI(robot_control::ModifyROI::Request &req, robot_control::ModifyROI::Response &res);
	bool searchMapCallback(robot_control::SearchMap::Request &req, robot_control::SearchMap::Response &res);
	bool globalMapPathHazardsCallback(messages::MapPathHazards::Request &req, messages::MapPathHazards::Response &res);
	bool searchLocalMapPathHazardsCallback(messages::MapPathHazards::Request &req, messages::MapPathHazards::Response &res);
	bool searchLocalMapInfoCallback(messages::SearchLocalMapInfo::Request &req, messages::SearchLocalMapInfo::Response &res);
	bool randomSearchWaypointsCallback(robot_control::RandomSearchWaypoints::Request &req, robot_control::RandomSearchWaypoints::Response &res);
	bool globalMapFullCallback(messages::GlobalMapFull::Request &req, messages::GlobalMapFull::Response &res);
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
	void setStartingPlatform();
	void calculateGlobalMapSize();
	void cutOutGlobalSubMap();
	// Members
	ros::NodeHandle nh;
	ros::ServiceServer roiServ;
	ros::ServiceServer modROIServ;
	ros::ServiceServer searchMapServ;
	ros::ServiceServer globalMapPathHazardsServ;
	ros::ServiceServer searchLocalMapPathHazardsServ;
	ros::ServiceServer searchLocalMapInfoServ;
	ros::ServiceServer randomSearchWaypointsServ;
	ros::ServiceServer globalMapFullServ;
	ros::ServiceClient createROIHazardMapClient;
	ros::Subscriber keyframesSub;
	ros::Subscriber globalPoseSub;
	ros::Subscriber keyframeRelPoseSub;
	ros::Subscriber cvSamplesFoundSub;
	ros::Publisher currentROIPub;
	ros::Publisher globalMapPub;
	ros::Publisher searchLocalMapPub;
	ros::Publisher globalSubMapPub;
	grid_map_msgs::GridMap globalMapMsg;
	grid_map_msgs::GridMap searchLocalMapMsg;
	grid_map_msgs::GridMap globalSubMapMsg;
	robot_control::Waypoint waypoint;
	robot_control::ROI ROI;
	std::vector<robot_control::ROI> regionsOfInterest;
	grid_map::GridMap globalMap;
	grid_map::Length globalMapSize;
	grid_map::Length satMapSize;
	grid_map::Size globalMapRowsCols;
	grid_map::GridMap globalSubMap;
	grid_map::GridMap globalMapTemp;
	grid_map::GridMap searchLocalMap;
	int searchLocalMapROINum;
	bool searchLocalMapExists;
	float searchLocalMapXPos;
	float searchLocalMapYPos;
	float searchLocalMapHeading;
	grid_map::Polygon globalMapPathHazardsPolygon;
	grid_map::Polygon searchLocalMapPathHazardsPolygon;
	std::vector<grid_map::Position> mapPathHazardsVertices;
	float mapPathHazardsPolygonHeading;
	float mapPathHazardValue;
	float mapPathHazardHeight;
	unsigned int mapPathHazardNumCellsInPolygon;
	grid_map::Position mapPathHazardPosition;
	messages::RobotPose globalPose;
	float previousNorthAngle; // deg
	messages::SLAMPoseOut keyframeRelPose;
	messages::KeyframeList keyframes;
	messages::CreateROIHazardMap createROIHazardMapSrv;
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
	grid_map::Position globalMapToSatMapPos;
	//float globalYTransformPos;
	robot_control::CurrentROI currentROIMsg;
	messages::CVSamplesFound cvSamplesFoundMsg;
	grid_map::Position globalMapOrigin;
	std::vector<float> possibleRandomWaypointValues;
	float possibleRandomWaypointValuesSum;
	std::vector<float> possibleRandomWaypointValuesNormalized;
	grid_map::Position randomWaypointPosition;
	grid_map::Position globalPoseToSearchLocalMapPosition;
	grid_map::Index randomWaypointIndex;
	float randomValue;
	float randomValueFloor;
	int searchLocalMapNumPoints;
	int candidateRandomWaypointIndex;
	int numRandomWaypointsToSelect;
	int numRandomWaypointsSelected;
	bool randomWaypointDistanceCriteriaFailed;
	int numRandomWaypointSearchDistanceCriteriaFailed;
	const int randomWaypointDistanceCriteriaFailedLimit = 100;
	const float mapResolution = 1.0; // m
	const float searchLocalMapLength = 40.0; // m
	const float searchLocalMapWidth = 40.0; // m
	const float sampleProbPeak = 1.0;
	const int smoothDriveabilityNumNeighborsToChangeValue = 6;
	const float randomWaypointMinDistance = 2.0; // m
	const float satDriveabilityInitialValue = 100.0;
	const float satDriveabilityInitialConf = 0.5;
	const float keyframeDriveabilityInitialValue = 0.0;
	const float keyframeDriveabilityInitialConf = 0.0;
	const float keyframeSize = 80.0;
	const unsigned int maxNormalWaypointAvoidCount = 3;
	arma::Mat<float> distanceMat;
	int startingPlatformLocation;
	float satMapStartE;
	float satMapStartS;
};

#endif // MAP_MANAGER_H
