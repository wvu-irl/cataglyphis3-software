#include <robot_control/safe_pathing.h>

SafePathing::SafePathing()
{
	ppServ = nh.advertiseService("/control/safepathing/intermediatewaypoints", &SafePathing::FindPath, this);
    robotPoseSub = nh.subscribe<messages::RobotPose>("/hsm/masterexec/globalpose", 1, &SafePathing::robotPoseCallback, this);
    startRadialDistance = 0.0;
    finishRadialDistance = 0.0;
    transitionWaypoint1.x = 8.0;
    transitionWaypoint1.y = 5.0;
    transitionWaypoint1.sampleProb = 0.0;
    transitionWaypoint1.terrainHazard = 0.0;
    transitionWaypoint1.searchable = false;
    transitionWaypoint2.x = 15.2711;
    transitionWaypoint2.y = 17.9457;
    transitionWaypoint2.sampleProb = 0.0;
    transitionWaypoint2.terrainHazard = 0.0;
    transitionWaypoint2.searchable = false;
    transitionWaypoint3.x = 31.9061;
    transitionWaypoint3.y = 22.1341;
    transitionWaypoint3.sampleProb = 0.0;
    transitionWaypoint3.terrainHazard = 0.0;
    transitionWaypoint3.searchable = false;
}

bool SafePathing::FindPath(robot_control::IntermediateWaypoints::Request &req, robot_control::IntermediateWaypoints::Response &res)
{
	intermediateWaypoints.clear();

    if(req.collision!=0 && req.collisionDistance<minCollisionDistance) // Check if collision distance is less than the minimum. If it is, set it to the minimum.
    {
        req.collisionDistance = minCollisionDistance;
    }
	if(req.collision==1) //object on left
	{
		const float turn_angle = 30; //turn 30 degrees (to right)
		//avoidance waypoint
        waypoint.x = req.current_x + req.collisionDistance*(cos(turn_angle*3.14159265/180)*cos(req.current_heading*3.14159265/180)-sin(turn_angle*3.14159265/180)*sin(req.current_heading*3.14159265/180)); //turn 30 deg drive 4 meters
        waypoint.y = req.current_y + req.collisionDistance*(cos(turn_angle*3.14159265/180)*sin(req.current_heading*3.14159265/180)+sin(turn_angle*3.14159265/180)*cos(req.current_heading*3.14159265/180)); //turn 30 deg drive 4 meters
        waypoint.sampleProb = 0.0;
		waypoint.terrainHazard = 0.0;
		intermediateWaypoints.push_back(waypoint);	
	}
	else if(req.collision==2) //object on right
	{
		const float turn_angle = -30; //turn 30 degrees (to left)
		//avoidance waypoint
        waypoint.x = req.current_x + req.collisionDistance*(cos(turn_angle*3.14159265/180)*cos(req.current_heading*3.14159265/180)-sin(turn_angle*3.14159265/180)*sin(req.current_heading*3.14159265/180)); //turn 30 deg drive 4 meters
        waypoint.y = req.current_y + req.collisionDistance*(cos(turn_angle*3.14159265/180)*sin(req.current_heading*3.14159265/180)+sin(turn_angle*3.14159265/180)*cos(req.current_heading*3.14159265/180)); //turn 30 deg drive 4 meters
        waypoint.sampleProb = 0.0;
		waypoint.terrainHazard = 0.0;
		intermediateWaypoints.push_back(waypoint);	
	}
    else if(req.collision==0) // No collision
    {
        startRadialDistance = hypot(req.start.x, req.start.y);
        finishRadialDistance = hypot(req.finish.x, req.finish.y);
        if(finishRadialDistance>=transitionWaypointRadius && startRadialDistance<transitionWaypointRadius)
        {
            intermediateWaypoints.push_back(transitionWaypoint1);
            intermediateWaypoints.push_back(transitionWaypoint2);
            intermediateWaypoints.push_back(transitionWaypoint3);
        }
        else if(finishRadialDistance<transitionWaypointRadius && startRadialDistance>=transitionWaypointRadius)
        {
            intermediateWaypoints.push_back(transitionWaypoint3);
            intermediateWaypoints.push_back(transitionWaypoint2);
            intermediateWaypoints.push_back(transitionWaypoint1);
        }
    }

	res.waypointArray = intermediateWaypoints;
	return true;
}

void SafePathing::robotPoseCallback(const messages::RobotPose::ConstPtr &msg)
{
    globalPose = *msg;
    if(globalPose.northAngle != northAnglePrev)
    {
        rotateCoord(transitionWaypoint2.x, transitionWaypoint2.y, transitionWaypoint2.x, transitionWaypoint2.y, globalPose.northAngle-90.0);
        rotateCoord(transitionWaypoint3.x, transitionWaypoint3.y, transitionWaypoint3.x, transitionWaypoint3.y, globalPose.northAngle-90.0);
    }
    northAnglePrev = globalPose.northAngle;
}

void SafePathing::rotateCoord(float origX, float origY, float &newX, float &newY, float angleDeg)
{
    newX = origX*cos(DEG2RAD*angleDeg)+origY*sin(DEG2RAD*angleDeg);
    newY = -origX*sin(DEG2RAD*angleDeg)+origY*cos(DEG2RAD*angleDeg);
}
