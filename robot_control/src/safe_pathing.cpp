#include <robot_control/safe_pathing.h>

SafePathing::SafePathing()
{
	ppServ = nh.advertiseService("/control/safepathing/intermediatewaypoints", &SafePathing::FastMarchingMethod, this);
}

bool SafePathing::FastMarchingMethod(robot_control::IntermediateWaypoints::Request &req, robot_control::IntermediateWaypoints::Response &res)
{
	intermediateWaypoints.clear();
	
	//first waypoint
	waypoint.x = 100.1;
	waypoint.y = 200.1;
	waypoint.easyProb = 0.0;
	waypoint.medProb = 0.0;
	waypoint.hardProb = 0.0;
	waypoint.terrainHazard = 0.0;
	intermediateWaypoints.push_back(waypoint);

	//middle waypoint 1
	waypoint.x = 101.1;
	waypoint.y = 201.1;
	waypoint.easyProb = 0.0;
	waypoint.medProb = 0.0;
	waypoint.hardProb = 0.0;
	waypoint.terrainHazard = 0.0;
	intermediateWaypoints.push_back(waypoint);

	//middle waypoint 2
	waypoint.x = 102.1;
	waypoint.y = 202.1;
	waypoint.easyProb = 0.0;
	waypoint.medProb = 0.0;
	waypoint.hardProb = 0.0;
	waypoint.terrainHazard = 0.0;
	intermediateWaypoints.push_back(waypoint);

	//last waypoint
	waypoint.x = 103.1;
	waypoint.y = 203.1;
	waypoint.easyProb = 0.0;
	waypoint.medProb = 0.0;
	waypoint.hardProb = 0.0;
	waypoint.terrainHazard = 0.0;
	intermediateWaypoints.push_back(waypoint);

	res.waypointArray = intermediateWaypoints;
	return true;
}
