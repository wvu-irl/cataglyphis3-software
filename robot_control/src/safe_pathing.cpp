#include <robot_control/safe_pathing.h>

SafePathing::SafePathing()
{
	ppServ = nh.advertiseService("/control/safepathing/intermediatewaypoints", &SafePathing::FindPath, this);
}

bool SafePathing::FindPath(robot_control::IntermediateWaypoints::Request &req, robot_control::IntermediateWaypoints::Response &res)
{
	intermediateWaypoints.clear();

	if(req.collision==1) //object on left
	{
		const float turn_angle = 30; //turn 30 degrees (to right)
		//avoidance waypoint
		waypoint.x = req.current_x + req.collisionDistance*cos(req.current_heading + turn_angle*3.14159265/180); //turn 30 deg drive 4 meters
		waypoint.y = req.current_y + req.collisionDistance*sin(req.current_heading + turn_angle*3.14159265/180); //turn 30 deg drive 4 meters
		waypoint.easyProb = 0.0;
		waypoint.medProb = 0.0;
		waypoint.hardProb = 0.0;
		waypoint.terrainHazard = 0.0;
		intermediateWaypoints.push_back(waypoint);	
	}
	else if(req.collision==2) //object on right
	{
		const float turn_angle = -30; //turn 30 degrees (to left)
		//avoidance waypoint
		waypoint.x = req.current_x + req.collisionDistance*cos(req.current_heading + turn_angle*3.14159265/180); //turn 30 deg drive 4 meters
		waypoint.y = req.current_y + req.collisionDistance*sin(req.current_heading + turn_angle*3.14159265/180); //turn 30 deg drive 4 meters
		waypoint.easyProb = 0.0;
		waypoint.medProb = 0.0;
		waypoint.hardProb = 0.0;
		waypoint.terrainHazard = 0.0;
		intermediateWaypoints.push_back(waypoint);	
	}

	res.waypointArray = intermediateWaypoints;
	return true;
}
