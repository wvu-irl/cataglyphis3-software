#include <robot_control/safe_pathing.h>

SafePathing::SafePathing()
        : mapOrigin(0.0, 0.0)
{
	ppServ = nh.advertiseService("/control/safepathing/intermediatewaypoints", &SafePathing::FindPath, this);
    robotPoseSub = nh.subscribe<messages::RobotPose>("/hsm/masterexec/globalpose", 1, &SafePathing::robotPoseCallback, this);
    globalMapFullClient = nh.serviceClient<messages::GlobalMapFull>("/control/mapmanager/globalmapfull");
    vizMapPub = nh.advertise<grid_map_msgs::GridMap>("/control/safepathing/mapviz", 1);
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
    vizMap.setFrameId("map");
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
        res.waypointArrayOut = req.waypointArrayIn;
        origNumWaypointsIn = req.waypointArrayIn.size();
        numInsertedWaypoints = 0;
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
        chooseWaypointsFromOptimalPath();
        res.waypointArrayOut.insert(res.waypointArrayOut.begin()+1+i+numInsertedWaypoints, waypointsToInsert.begin(), waypointsToInsert.end());
        res.waypointArrayOut.erase(res.waypointArrayOut.begin()); // Do not send current location as a waypoint to travel*/
    }
    else if(req.collision==0) // No collision
    {
        waypoint.x = req.current_x;
        waypoint.y = req.current_y;
        req.waypointArrayIn.insert(req.waypointArrayIn.begin(),waypoint);
        res.waypointArrayOut = req.waypointArrayIn;
        origNumWaypointsIn = req.waypointArrayIn.size();
        if(origNumWaypointsIn>0)
        {
            numInsertedWaypoints = 0;
            if(globalMapFullClient.call(globalMapFullSrv)) ROS_INFO("globalMapFull service call successful");
            else ROS_ERROR("globalMapFull service call unsuccessful");
            grid_map::GridMapRosConverter::fromMessage(globalMapFullSrv.response.globalMap, globalMap);
            mapDimensions = globalMap.getLength();
            timeOfArrivalMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
            initialViscosityMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
            resistanceMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
            ROS_INFO("before for loop");
            for(int i=0; i<(origNumWaypointsIn-1); i++)
            {
                timeOfArrivalMap.add(timeLayer, initialTimeValue);
                timeOfArrivalMap.add(setLayer, (float)_unknown);
                initialViscosityMap.add(timeLayer, initialTimeValue);
                initialViscosityMap.add(setLayer, (float)_unknown);
                resistanceMap.add(timeLayer, initialTimeValue);
                resistanceMap.add(setLayer, (float)_unknown);
                startPosition[0] = req.waypointArrayIn.at(i).x;
                startPosition[1] = req.waypointArrayIn.at(i).y;
                goalPoint[0] = req.waypointArrayIn.at(i+1).x;
                goalPoint[1] = req.waypointArrayIn.at(i+1).y;
                ROS_INFO("before resistanceMap FMM");
                FMM(initialViscosityMap, resistanceMap, goalPoint);
                ROS_INFO("before write sat map into resistance map");
                // resistanceMap + satMap
                /*for(grid_map::GridMapIterator it(resistanceMap); !it.isPastEnd(); ++it)
                {
                    resistanceMap.at(timeLayer, *it) += globalMap.at(layerToString(_satDriveability), *it); // Just add or average?
                    if(resistanceMap.at(timeLayer, *it) > 10.0) resistanceMap.at(timeLayer, *it) = 10.0;
                }*/
                ROS_INFO("before timeOfArrivalMap FMM");
                FMM(resistanceMap, timeOfArrivalMap, goalPoint);
                ROS_INFO("before gradientDescent");
                ROS_INFO("optimalPath.size() = %u",optimalPath.size());
                gradientDescent(timeOfArrivalMap, startPosition, optimalPath);
                ROS_INFO("before chooseWaypointsFromOptimalPath");
                ROS_INFO("optimalPath.size() = %u",optimalPath.size());
                chooseWaypointsFromOptimalPath();
                ROS_INFO("before waypointsOut.insert");
                ROS_INFO("optimalPath.size() = %u",optimalPath.size());
                if(waypointsToInsert.size()>0)
                    res.waypointArrayOut.insert(res.waypointArrayOut.begin()+1+i+numInsertedWaypoints, waypointsToInsert.begin(), waypointsToInsert.end());
                ROS_INFO("before generateAngPubVizMap");
                generateAndPubVizMap();
            }
            ROS_INFO("before waypointsOut.erase(begin)");
            res.waypointArrayOut.erase(res.waypointArrayOut.begin()); // Do not send current location as a waypoint to travel
        }
        else ROS_ERROR("called intermediate waypoints with less no waypointsIn");

        /*waypoint.x = req.current_x;
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
        res.waypointArrayOut.erase(res.waypointArrayOut.begin()); // Do not send current location as a waypoint to travel*/
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

void SafePathing::FMM(grid_map::GridMap &mapIn, grid_map::GridMap &mapOut, grid_map::Position &goalPointIn)
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
    mapOut.atPosition(timeLayer, goalPointIn) = 0.0;
    mapOut.atPosition(setLayer, goalPointIn) = (float)_narrowBand;
    mapOut.getIndex(goalPointIn, currentIndex);
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

void SafePathing::chooseWaypointsFromOptimalPath()
{
    grid_map::Index startIndex;
    grid_map::Index indexToConsider;
    grid_map::Index prevIndex;
    grid_map::Position startIndexPos;
    grid_map::Position considerIndexPos;
    grid_map::Position prevIndexPos;
    float distanceBetweenIndices;
    float hazardAlongPossiblePathThresh;
    unsigned int numCellsOverThresh;
    grid_map::Polygon mapPolygon;
    std::vector<grid_map::Position> polygonVertices;
    float mapPolygonHeading;
    bool continueLoop = true;
    robot_control::Waypoint waypointToPushBack;

    waypointsToInsert.clear();
    startIndex = optimalPath.front();
    indexToConsider = optimalPath.back();
    numCellsOverThresh = 0;
    while(continueLoop)
    {
        hazardAlongPossiblePathThresh = initialHazardAlongPossiblePathThresh;
        timeOfArrivalMap.getPosition(startIndex, startIndexPos);
        timeOfArrivalMap.getPosition(indexToConsider, considerIndexPos);
        if(hypot(considerIndexPos[0]-startIndexPos[0], considerIndexPos[1]-startIndexPos[1]) < minWaypointDistance)
        {
            continueLoop = false;
            break;
        }
        mapPolygon.removeVertices();
        mapPolygonHeading = atan2(considerIndexPos[1] - startIndexPos[1], considerIndexPos[0] - startIndexPos[0]); // radians
        polygonVertices.at(0)[0] = startIndexPos[0] + corridorWidth/2.0*sin(mapPolygonHeading);
        polygonVertices.at(0)[1] = startIndexPos[1] - corridorWidth/2.0*cos(mapPolygonHeading);
        polygonVertices.at(1)[0] = startIndexPos[0] - corridorWidth/2.0*sin(mapPolygonHeading);
        polygonVertices.at(1)[1] = startIndexPos[1] + corridorWidth/2.0*cos(mapPolygonHeading);
        polygonVertices.at(2)[0] = considerIndexPos[0] - corridorWidth/2.0*sin(mapPolygonHeading);
        polygonVertices.at(2)[1] = considerIndexPos[1] + corridorWidth/2.0*cos(mapPolygonHeading);
        polygonVertices.at(3)[0] = considerIndexPos[0] + corridorWidth/2.0*sin(mapPolygonHeading);
        polygonVertices.at(3)[1] = considerIndexPos[1] - corridorWidth/2.0*cos(mapPolygonHeading);
        mapPolygon.addVertex(polygonVertices.at(0));
        mapPolygon.addVertex(polygonVertices.at(1));
        mapPolygon.addVertex(polygonVertices.at(2));
        mapPolygon.addVertex(polygonVertices.at(3));
        for(grid_map::PolygonIterator it(timeOfArrivalMap, mapPolygon); !it.isPastEnd(); ++it)
        {
            if(timeOfArrivalMap.at(timeLayer, *it) > hazardAlongPossiblePathThresh) numCellsOverThresh++;
        }
        if(numCellsOverThresh>=numCellsOverThreshLimit)
        {
            prevIndex = indexToConsider;
            for(int i=optimalPath.size()-1; i>=0; i--)
            {
                indexToConsider = optimalPath.at(i);
                timeOfArrivalMap.getPosition(indexToConsider, considerIndexPos);
                timeOfArrivalMap.getPosition(prevIndex, prevIndexPos);
                distanceBetweenIndices = hypot(considerIndexPos[0]-prevIndexPos[0], considerIndexPos[1]-prevIndexPos[1]);
                if(indexToConsider[0]==optimalPath.front()[0] && indexToConsider[1]==optimalPath.front()[1]) hazardAlongPossiblePathThresh += hazardThreshIncrementAmount;
                else if(distanceBetweenIndices>minWaypointDistance)
                {
                    numCellsOverThresh = 0;
                    break;
                }
            }
            continueLoop = true;
        }
        else
        {
            waypointToPushBack.x = considerIndexPos[0];
            waypointToPushBack.y = considerIndexPos[1];
            waypointsToInsert.push_back(waypointToPushBack);
            if(indexToConsider[0]==optimalPath.back()[0] && indexToConsider[1]==optimalPath.back()[1]) continueLoop = false;
            else {startIndex = indexToConsider; continueLoop = true;}
        }
    }
}

void SafePathing::generateAndPubVizMap()
{
    ROS_INFO("before setGeometry");
    vizMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
    ROS_INFO("before add layers");
    vizMap.add(vizResistanceLayer, 0.0);
    vizMap.add(vizTimeOfArrivalLayer, 0.0);
    vizMap.add(vizOptimalPathLayer, 0.0);
    ROS_INFO("before iterator loop");
    for(grid_map::GridMapIterator it(vizMap); !it.isPastEnd(); ++it)
    {
        vizMap.at(vizResistanceLayer, *it) = resistanceMap.at(timeLayer, *it);
        vizMap.at(vizTimeOfArrivalLayer, *it) = timeOfArrivalMap.at(timeLayer, *it);
    }
    /*ROS_INFO("before optimalPath loop");
    ROS_INFO("optimalPath.size() = %u",optimalPath.size());
    for(int i=0; i<optimalPath.size(); i++)
    {
        ROS_INFO("i = %i", i);
        ROS_INFO("optimalPath[i] = (%i,%i)",optimalPath.at(i)[0],optimalPath.at(i)[1]);
        vizMap.at(vizOptimalPathLayer, optimalPath.at(i)) = 1.0;
    }*/
    ROS_INFO("before toMessage");
    grid_map::GridMapRosConverter::toMessage(vizMap, vizMapMsg);
    ROS_INFO("before publish");
    vizMapPub.publish(vizMapMsg);
}
