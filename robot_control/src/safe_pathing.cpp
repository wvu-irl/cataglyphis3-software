#include <robot_control/safe_pathing.h>

SafePathing::SafePathing()
        : mapOrigin(0.0, 0.0)
{
	ppServ = nh.advertiseService("/control/safepathing/intermediatewaypoints", &SafePathing::FindPath, this);
    robotPoseSub = nh.subscribe<messages::RobotPose>("/hsm/masterexec/globalpose", 1, &SafePathing::robotPoseCallback, this);
    globalMapFullClient = nh.serviceClient<messages::GlobalMapFull>("/control/mapmanager/globalmapfull");
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
    mapDimensions[0] = 500;
    mapDimensions[1] = 500;
    timeOfArrivalMap.setFrameId("map");
    initialViscosityMap.setFrameId("map");
    resistanceMap.setFrameId("map");

}

bool SafePathing::FindPath(robot_control::IntermediateWaypoints::Request &req, robot_control::IntermediateWaypoints::Response &res)
{
    res.waypointArrayOut.clear();

    if((req.collision==1 || req.collision==2) && req.collisionDistance<minCollisionDistance) // Check if collision distance is less than the minimum. If it is, set it to the minimum.
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
        res.waypointArrayOut.resize(1);
        res.waypointArrayOut.at(0) = waypoint;
	}
	else if(req.collision==2) //object on right
	{
		const float turn_angle = -30; //turn 30 degrees (to left)
		//avoidance waypoint
        waypoint.x = req.current_x + req.collisionDistance*(cos(turn_angle*3.14159265/180)*cos(req.current_heading*3.14159265/180)-sin(turn_angle*3.14159265/180)*sin(req.current_heading*3.14159265/180)); //turn 30 deg drive 4 meters
        waypoint.y = req.current_y + req.collisionDistance*(cos(turn_angle*3.14159265/180)*sin(req.current_heading*3.14159265/180)+sin(turn_angle*3.14159265/180)*cos(req.current_heading*3.14159265/180)); //turn 30 deg drive 4 meters
        waypoint.sampleProb = 0.0;
		waypoint.terrainHazard = 0.0;
        res.waypointArrayOut.resize(1);
        res.waypointArrayOut.at(0) = waypoint;
	}
    else if(req.collision==3) // Predictive avoidance
    {
        /*waypoint.x = req.current_x;
        waypoint.y = req.current_y;
        req.waypointArrayIn.insert(req.waypointArrayIn.begin(),waypoint);
        if(globalMapFullClient.call(globalMapFullSrv)) ROS_DEBUG("globalMapFull service call successful");
        else ROS_ERROR("globalMapFull service call unsuccessful");
        grid_map::GridMapRosConverter::fromMessage(globalMapFullSrv.response.globalMap, globalMap);
        // Call service to get local hazard map and then merge it into the global map
        mapDimensions = globalMap.getLength();
        timeOfArrivalMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
        initialViscosityMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
        resistanceMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
        timeOfArrivalMap.add(timeLayer, initialTimeValue);
        timeOfArrivalMap.add(setLayer, (float)_unknown);
        initialViscosityMap.add(timeLayer, initialTimeValue);
        initialViscosityMap.add(setLayer, (float)_unknown);
        resistanceMap.add(timeLayer, initialTimeValue);
        resistanceMap.add(setLayer, (float)_unknown);
        FMM(initialViscosityMap, resistanceMap, goalPoints...);
        // resistanceMap + satMap + localHazardMap
        FMM(resistanceMap, timeOfArrivalMap, finalDestination...);
        gradientDescent(timeOfArrivalMap, startPosition, optimalPath);
        // Back solve on optimal path
        res.waypointArrayOut.erase(res.waypointArrayOut.begin()); // Do not send current location as a waypoint to travel*/
    }
    else if(req.collision==0) // No collision
    {
        /*waypoint.x = req.current_x;
        waypoint.y = req.current_y;
        req.waypointArrayIn.insert(req.waypointArrayIn.begin(),waypoint);
        startPosition[0] = req.current_x;
        startPosition[1] = req.current_y;
        if(globalMapFullClient.call(globalMapFullSrv)) ROS_DEBUG("globalMapFull service call successful");
        else ROS_ERROR("globalMapFull service call unsuccessful");
        grid_map::GridMapRosConverter::fromMessage(globalMapFullSrv.response.globalMap, globalMap);
        mapDimensions = globalMap.getLength();
        timeOfArrivalMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
        initialViscosityMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
        resistanceMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
        timeOfArrivalMap.add(timeLayer, initialTimeValue);
        timeOfArrivalMap.add(setLayer, (float)_unknown);
        initialViscosityMap.add(timeLayer, initialTimeValue);
        initialViscosityMap.add(setLayer, (float)_unknown);
        resistanceMap.add(timeLayer, initialTimeValue);
        resistanceMap.add(setLayer, (float)_unknown);
        goalPoints.resize(req.waypointArrayIn.size());
        finalDestination.resize(1);
        finalDestination.back()[0] = req.waypointArrayIn.back().x;
        finalDestination.back()[1] = req.waypointArrayIn.back().y;
        for(int i=0; i<req.waypointArrayIn.size(); i++)
        {
            goalPoints.at(i)[0] = req.waypointArrayIn.at(i).x;
            goalPoints.at(i)[1] = req.waypointArrayIn.at(i).y;
        }
        FMM(initialViscosityMap, resistanceMap, goalPoints);
        // resistanceMap + satMap
        for(grid_map::GridMapIterator it(resistanceMap); !it.isPastEnd(); ++it)
        {
            resistanceMap.at(timeLayer, *it) += globalMap.at(layerToString(_satDriveability), *it); // Just add or average?
            if(resistanceMap.at(timeLayer, *it) > 10.0) resistanceMap.at(timeLayer, *it) = 10.0;
        }
        FMM(resistanceMap, timeOfArrivalMap, finalDestination);
        gradientDescent(timeOfArrivalMap, startPosition, optimalPath);
        // Back solve on optimal path
        res.waypointArrayOut.erase(res.waypointArrayOut.begin()); // Do not send current location as a waypoint to travel*/

        waypoint.x = req.current_x;
        waypoint.y = req.current_y;
        req.waypointArrayIn.insert(req.waypointArrayIn.begin(),waypoint);
        res.waypointArrayOut = req.waypointArrayIn;
        origNumWaypointsIn = req.waypointArrayIn.size();
        numInsertedWaypoints = 0;
        for(int i=0; i<(origNumWaypointsIn-1); i++)
        {
            startRadialDistance = hypot(req.waypointArrayIn.at(i).x, req.waypointArrayIn.at(i).y);
            finishRadialDistance = hypot(req.waypointArrayIn.at(i+1).x, req.waypointArrayIn.at(i+1).y);
            if(finishRadialDistance>=transitionWaypointOuterRadius && startRadialDistance<transitionWaypointInnerRadius)
            {
                ROS_INFO("going out transition waypoints");
                res.waypointArrayOut.insert(res.waypointArrayOut.begin() + i+1+numInsertedWaypoints, transitionWaypoint1);
                res.waypointArrayOut.insert(res.waypointArrayOut.begin() + i+2+numInsertedWaypoints, transitionWaypoint2);
                res.waypointArrayOut.insert(res.waypointArrayOut.begin() + i+3+numInsertedWaypoints, transitionWaypoint3);
                numInsertedWaypoints+=3;
                //intermediateWaypoints.push_back(transitionWaypoint1);
                //intermediateWaypoints.push_back(transitionWaypoint2);
                //intermediateWaypoints.push_back(transitionWaypoint3);
            }
            else if(finishRadialDistance<transitionWaypointInnerRadius && startRadialDistance>=transitionWaypointOuterRadius)
            {
                ROS_INFO("going back transition waypoints");
                res.waypointArrayOut.insert(res.waypointArrayOut.begin() + i+1+numInsertedWaypoints, transitionWaypoint3);
                res.waypointArrayOut.insert(res.waypointArrayOut.begin() + i+2+numInsertedWaypoints, transitionWaypoint2);
                res.waypointArrayOut.insert(res.waypointArrayOut.begin() + i+3+numInsertedWaypoints, transitionWaypoint1);
                numInsertedWaypoints+=3;
                //intermediateWaypoints.push_back(transitionWaypoint3);
                //intermediateWaypoints.push_back(transitionWaypoint2);
                //intermediateWaypoints.push_back(transitionWaypoint1);
            }
        }
        res.waypointArrayOut.erase(res.waypointArrayOut.begin()); // Do not send current location as a waypoint to travel
    }

    //res.waypointArrayOut = intermediateWaypoints;
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
    if(goalPointsIn.size()<2)
    {
        ROS_ERROR("number of goal points passed into FMM less than 2");
    }
    else
    {
        for(int k=1; k<goalPointsIn.size(); k++)
        {
            mapOut.atPosition(timeLayer, goalPointsIn.at(k)) = 0.0;
            mapOut.atPosition(setLayer, goalPointsIn.at(k)) = (float)_narrowBand;
            mapOut.getIndex(goalPointsIn.at(k), currentIndex);
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
                                    if(mapOut.at(setLayer, offsetIndex)==(float)_unknown) mapOut.at(setLayer, offsetIndex) = (float)_narrowBand;

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
