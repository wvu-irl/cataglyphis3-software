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
#include "map_layers.h"
#include <messages/GlobalMapFull.h>
#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI

enum SET_LAYER_T {_frozen, _narrowBand, _unknown};

class SafePathing
{
public:
	// Methods
	SafePathing(); // Constructor
	bool FindPath(robot_control::IntermediateWaypoints::Request &req, robot_control::IntermediateWaypoints::Response &res);
	void robotPoseCallback(const messages::RobotPose::ConstPtr& msg);
	void rotateCoord(float origX, float origY, float &newX, float &newY, float angleDeg);
	void FMM(grid_map::GridMap& mapIn, grid_map::GridMap& mapOut, std::vector<grid_map::Position>& goalPointsIn);
	bool narrowBandNotEmpty(grid_map::GridMap& map);
	void gradientDescent(grid_map::GridMap& map, grid_map::Position startPosition, std::vector<grid_map::Index>& pathOut);
	// Members
	ros::NodeHandle nh;
	ros::ServiceServer ppServ;
	ros::Subscriber robotPoseSub;
	ros::ServiceClient globalMapFullClient;
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
	const float initialTimeValue = 0.1;
	grid_map::GridMap resistanceMap;
	grid_map::GridMap globalMap;
	std::vector<grid_map::Position> goalPoints;
	const std::string timeLayer = "timeLayer";
	const std::string setLayer = "setLayer"; // 0 = frozen, 1 = narrow band, 2 = unknown
	const float mapResolution = 1.0; // m
	grid_map::Length mapDimensions;
	const grid_map::Position mapOrigin;
};

#endif // SAFE_PATHING_H 
