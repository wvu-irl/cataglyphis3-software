#include <robot_control/safe_pathing.h>

SafePathing::SafePathing()
        : mapOrigin(0.0, 0.0)
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
    timeOfArrivalMap.setFrameId("map");

    /*initialViscosityMap.setFrameId("map");
    initialViscosityMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
    initialViscosityMap.add(timeLayer, initialViscosityValue);
    resistanceMap.setFrameId("map");
    resistanceMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
    resistanceMap.add(timeLayer, initialViscosityValue);*/

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

    /*FMM(initialViscosityMap, resistanceMap, goalPoints...);
    // resistanceMap + satMap
    FMM(resistanceMap, timeOfArrivalMap, finalDestination...);
    gradientDescent(timeOfArrivalMap, startPosition..., optimalPath);
    // Back solve on optimal path*/

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

void SafePathing::FMM(grid_map::GridMap &mapIn, grid_map::GridMap &mapOut, std::vector<grid_map::Position> &goalPointsIn)
{
    float delta;
    float dx;
    float dy;
    bool continueLoop;
    float bestNarrowBandValue;
    grid_map::Index bestNarrowBandIndex;
    grid_map::Size mapSize;
    grid_map::Index currentIndex;
    grid_map::Index offsetIndex;
    //mapOut.add(timeLayer, 10.0);
    mapOut.add(setLayer, (float)_unknown);
    mapSize = mapOut.getSize();
    for(int k=0; k<goalPointsIn.size(); k++)
    {
        mapOut.atPosition(timeLayer, goalPointsIn.at(k)) = 0.0;
        mapOut.atPosition(setLayer, goalPointsIn.at(k)) = (float)_narrowBand;
        mapOut.getIndex(goalPointsIn.at(k), currentIndex);
        // Need to keep going until some termination condition.
        while(continueLoop)
        {
            continueLoop = narrowBandNotEmpty(mapOut)/* || currentIndex != startIndex*/;
            if(continueLoop)
            {
                bestNarrowBandValue = 10.0;
                for(grid_map::GridMapIterator it(mapOut); !it.isPastEnd(); ++it)
                {
                    if(mapOut.at(setLayer, *it) == (float)_narrowBand)
                    {
                        if(mapOut.at(timeLayer, *it) < bestNarrowBandValue)
                        {
                            bestNarrowBandValue = mapOut.at(timeLayer, *it);
                            bestNarrowBandIndex = grid_map::getIndexFromLinearIndex(it.getLinearIndex(), mapOut.getSize());
                        }
                        mapOut.at(setLayer, bestNarrowBandIndex) = (float)_frozen;
                    }
                }
                for(int i=-1; i<=1; i++)
                {
                    for(int j=-1; j<=1; j++)
                    {
                        if((i != 0 && j == 0) || (i == 0 && j != 0))
                        {
                            offsetIndex[0] = currentIndex[0] + i;
                            offsetIndex[1] = currentIndex[1] + j;
                            if((offsetIndex[0]<mapSize[0] && offsetIndex[0]>=0) && (offsetIndex[1]<mapSize[1] && offsetIndex[1]>=0))
                            {
                                if(mapOut.at(setLayer, offsetIndex)==(float)_unknown) mapOut.at(setLayer, offsetIndex) = (float)_narrowBand; // How to points get set to frozen other than being a goal point?

                                if(mapOut.get(timeLayer)(currentIndex[0]-1,currentIndex[1]) < mapOut.get(timeLayer)(currentIndex[0]+1,currentIndex[1])) dx = mapOut.get(timeLayer)(currentIndex[0]-1,currentIndex[1]);
                                else dx = mapOut.get(timeLayer)(currentIndex[0]+1,currentIndex[1]);

                                if(mapOut.get(timeLayer)(currentIndex[0],currentIndex[1]-1) < mapOut.get(timeLayer)(currentIndex[0],currentIndex[1]+1)) dy = mapOut.get(timeLayer)(currentIndex[0],currentIndex[1]-1);
                                else dy = mapOut.get(timeLayer)(currentIndex[0],currentIndex[1]+1);

                                delta = 2.0*mapIn.at(timeLayer, currentIndex) - pow(dx-dy, 2.0);
                                if(delta>=0.0) mapOut.at(timeLayer, currentIndex) = (dx+dy+sqrt(delta))/2.0;
                                else
                                {
                                    if(dx+mapIn.at(timeLayer, currentIndex) < dy+mapIn.at(timeLayer, currentIndex)) mapOut.at(timeLayer, currentIndex) = dx+mapIn.at(timeLayer, currentIndex);
                                    else mapOut.at(timeLayer, currentIndex) = dy+mapIn.at(timeLayer, currentIndex);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

bool SafePathing::narrowBandNotEmpty(grid_map::GridMap &map)
{
    for(grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
    {
        if(map.at(setLayer, *it)==(float)_narrowBand) return true;
    }
    return false;
}

void SafePathing::gradientDescent(grid_map::GridMap &map, grid_map::Position startPosition, std::vector<grid_map::Index> &pathOut)
{
    grid_map::Size mapSize;
    bool continueLoop = true;
    float nextBestValue = map.atPosition(timeLayer, startPosition); // Initialize nextBestValue to starting position value
    grid_map::Index currentIndex;
    grid_map::Index offsetIndex;
    grid_map::Index nextBestIndex;
    pathOut.clear();
    map.getIndex(startPosition, currentIndex);
    mapSize = map.getSize();
    int i, j;
    while(continueLoop)
    {
        for(i=-1; i<=1; i++)
        {
            for(j=-1; j<=1; j++)
            {
                if((i != 0 && j == 0) || (i == 0 && j != 0))
                {
                    offsetIndex[0] = currentIndex[0] + i;
                    offsetIndex[1] = currentIndex[1] + j;
                    if((offsetIndex[0]<mapSize[0] && offsetIndex[0]>=0) && (offsetIndex[1]<mapSize[1] && offsetIndex[1]>=0))
                    {
                        continueLoop = false;
                        if(map.at(timeLayer, offsetIndex) < nextBestValue)
                        {
                            nextBestValue = map.at(timeLayer, offsetIndex);
                            nextBestIndex = offsetIndex;
                            continueLoop = true;
                        }
                    }
                }
            }
        }
        currentIndex = nextBestIndex;
        pathOut.push_back(currentIndex);
    }
}
