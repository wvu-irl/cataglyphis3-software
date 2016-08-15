#ifndef SAFE_PATHING_H
#define SAFE_PATHING_H
#include <ros/ros.h>
#include <robot_control/Waypoint.h>
#include <robot_control/IntermediateWaypoints.h>
#include <messages/RobotPose.h>
#include <vector>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <math.h>
#include <set>
#include <iterator>
#include "map_layers.h"
#include <messages/GlobalMapFull.h>
#include <hsm/voice.h>
#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI

enum SET_LAYER_T {_frozen, _narrowBand, _unknown};
enum STRAIGHT_LINE_CONDITION_T {_driveable, _distanceTooLong, _tooManyObstacles};

class MapData
{
public:
	float value;
	grid_map::Index mapIndex;
};

struct MapDataLess
{
	bool operator() (const MapData &x, const MapData &y) const
	{
		return x.value < y.value;
	}
	typedef MapData first_argument_type;
	typedef MapData second_argument_type;
	typedef bool result_type;
};

class SafePathing
{
public:
	// Methods
	SafePathing(); // Constructor
	bool FindPath(robot_control::IntermediateWaypoints::Request &req, robot_control::IntermediateWaypoints::Response &res);
	void robotPoseCallback(const messages::RobotPose::ConstPtr& msg);
	void rotateCoord(float origX, float origY, float &newX, float &newY, float angleDeg);
	void FMM(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut, grid_map::Position &goalPointIn);
	bool narrowBandNotEmpty();
	void gradientDescent(grid_map::GridMap& map, grid_map::Position startPosition, std::vector<grid_map::Index>& pathOut);
	void chooseWaypointsFromOptimalPath();
	STRAIGHT_LINE_CONDITION_T straightLineDriveable(grid_map::GridMap& map, std::string layer, grid_map::Position& startPos, grid_map::Position& endPos, float hazardThresh, unsigned int numCellsLimit, bool useMinDistanceLimit);
	void transitionWaypoints(std::vector<robot_control::Waypoint>& waypointList);
	void generateAndPubVizMap(std::vector<robot_control::Waypoint> waypointList);
	void addToSet(std::multiset<MapData, MapDataLess>& set, MapData& cell);
	void removeFromSet(std::multiset<MapData, MapDataLess>& set, float cellValue, grid_map::Index mapIndex);
	void modifyValueOfIndexInSet(std::multiset<MapData, MapDataLess>& set, float oldCellValue, float newCellValue, grid_map::Index mapIndex);
	void swapCellInSet(std::multiset<MapData, MapDataLess>& fromSet, std::multiset<MapData, MapDataLess>& toSet, float cellValue, grid_map::Index mapIndex);
	//float getValueInSet();
	// Members
	ros::NodeHandle nh;
	ros::ServiceServer ppServ;
	ros::Subscriber robotPoseSub;
	ros::ServiceClient globalMapFullClient;
	ros::Publisher vizMapPub;
	grid_map_msgs::GridMap vizMapMsg;
	grid_map::GridMap vizMap;
	messages::GlobalMapFull globalMapFullSrv;
	messages::RobotPose globalPose;
	std::vector <robot_control::Waypoint> intermediateWaypoints;
	robot_control::Waypoint waypoint;
	robot_control::Waypoint transitionWaypoint1;
	robot_control::Waypoint transitionWaypoint2;
	robot_control::Waypoint transitionWaypoint3;
	const float transitionWaypointInnerRadius = 20.0; // m
	const float transitionWaypointOuterRadius = 30.0; // m
	const float minCollisionDistance = 2.0; // m
	float startRadialDistance;
	float finishRadialDistance;
	float northAnglePrev = 89.1; // deg
	grid_map::GridMap timeOfArrivalMap;
	std::vector<grid_map::Index> optimalPath;
	std::vector<grid_map::Position> hazardMapPoints;
	grid_map::GridMap initialViscosityMap;
	//const float initialTimeValue = 0.0005;
	const float initialTimeValue = 0.000002;
	const float maxTimeValue = 10.0;
	const float minFMMTimeValue = 0.0;
	grid_map::GridMap resistanceMap;
	grid_map::GridMap globalMap;
	grid_map::Position goalPoint;
	float globalMapValue;
	const float globalMapValueScaleFactor = 0.03; // 0.08
	MapData mapCell;
	std::multiset<MapData, MapDataLess> narrowBandSet;
	const std::string timeLayer = "timeLayer";
	const std::string setLayer = "setLayer"; // 0 = frozen, 1 = narrow band, 2 = unknown
	const std::string vizResistanceLayer = "resistance";
	const std::string vizTimeOfArrivalLayer = "timeOfArrival";
	const std::string vizOptimalPathLayer = "optimalPath";
	const float mapResolution = 1.0; // m
	grid_map::Length mapDimensions;
	const grid_map::Position mapOrigin;
	grid_map::Position startPosition;
	unsigned int origNumWaypointsIn;
	unsigned int numInsertedWaypoints;
	std::vector<robot_control::Waypoint> waypointsToInsert;
	STRAIGHT_LINE_CONDITION_T initialStraightLineCondition;
	float angleBetweenStartAndGoal; // rad
	bool stillInsertingWaypoints;
	grid_map::Position workingPos;
	robot_control::Waypoint insertWaypoint;
	const float initialHazardAlongPossiblePathThresh = 5.0;
	const float hazardThreshIncrementAmount = 1.0;
	const unsigned int numCellsOverThreshLimit = 8;
	const float straightLineCheckHazardThresh = 2.0;
	const unsigned int straightLineNumCellsOverThreshLimit = 5;
	const float minWaypointDistance = 5.0; // m
	const float corridorWidth = 2.0; // m
	const float minDriveDistanceForFMM = 10.0; // m
	const float maxDriveDistance = 35.0; // m
	const float homingRadius = 20.0; // m
	const float atHomeRadius = 10.0; // m
	robot_control::Waypoint quad1MagneticWaypoint;
	robot_control::Waypoint quad2MagneticWaypoint;
	robot_control::Waypoint quad3MagneticWaypoint;
	robot_control::Waypoint quad4MagneticWaypoint;
	robot_control::Waypoint homeWaypoint;
	Voice voiceSay;
};

#endif // SAFE_PATHING_H 
