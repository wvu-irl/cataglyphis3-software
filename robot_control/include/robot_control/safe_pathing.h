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
	void gradientDescent(grid_map::GridMap& map, grid_map::Position startPosition, std::vector<grid_map::Index>& pathOut);
	// Members
	ros::NodeHandle nh;
	ros::ServiceServer ppServ;
	ros::Subscriber robotPoseSub;
	messages::RobotPose globalPose;
	std::vector <robot_control::Waypoint> intermediateWaypoints;
	robot_control::Waypoint waypoint;
	robot_control::Waypoint transitionWaypoint1;
	robot_control::Waypoint transitionWaypoint2;
	robot_control::Waypoint transitionWaypoint3;
	const float transitionWaypointRadius = 30.0; // m
	const float minCollisionDistance = 2.0; // m
	float startRadialDistance;
	float finishRadialDistance;
	float northAnglePrev = 89.1; // deg
	grid_map::GridMap globalMap;
	std::vector<grid_map::Position> hazardMapPoints;
	const std::string timeLayer = "timeLayer";
	const std::string setLayer = "setLayer"; // 0 = frozen, 1 = narrow band, 2 = unknown
	const std::string viscosityLayer = "viscosityLayer";
};

#endif // SAFE_PATHING_H 
