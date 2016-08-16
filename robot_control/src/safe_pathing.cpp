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
    quad1MagneticWaypoint.x = 7.0;
    quad1MagneticWaypoint.y = 7.0;
    quad1MagneticWaypoint.unskippable = true;
    quad1MagneticWaypoint.roiWaypoint = false;
    quad2MagneticWaypoint.x = 7.0;
    quad2MagneticWaypoint.y = -7.0;
    quad2MagneticWaypoint.unskippable = true;
    quad2MagneticWaypoint.roiWaypoint = false;
    quad3MagneticWaypoint.x = -7.0;
    quad3MagneticWaypoint.y = -7.0;
    quad3MagneticWaypoint.unskippable = true;
    quad3MagneticWaypoint.roiWaypoint = false;
    quad4MagneticWaypoint.x = 7.0;
    quad4MagneticWaypoint.y = -7.0;
    quad4MagneticWaypoint.unskippable = true;
    quad4MagneticWaypoint.roiWaypoint = false;
    homeWaypoint.x = 5.0;
    homeWaypoint.y = 0.0;
    mapDimensions[0] = 500;
    mapDimensions[1] = 500;
    timeOfArrivalMap.setFrameId("map");
    initialViscosityMap.setFrameId("map");
    resistanceMap.setFrameId("map");
    vizMap.setFrameId("map");
    timeOfArrivalMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
    initialViscosityMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
    resistanceMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
    timeOfArrivalMap.add(timeLayer, maxTimeValue);
    timeOfArrivalMap.add(setLayer, (float)_unknown);
    initialViscosityMap.add(timeLayer, initialTimeValue);
    initialViscosityMap.add(setLayer, (float)_unknown);
    resistanceMap.add(timeLayer, initialTimeValue);
    resistanceMap.add(setLayer, (float)_unknown);
}

bool SafePathing::FindPath(robot_control::IntermediateWaypoints::Request &req, robot_control::IntermediateWaypoints::Response &res)
{
    double firsttime;
    double nexttime;
    double deltatime;
    res.waypointArrayOut.clear();

    if((req.collision==1 || req.collision==2) && req.collisionDistance<minCollisionDistance) // Check if collision distance is less than the minimum. If it is, set it to the minimum.
    {
        req.collisionDistance = minCollisionDistance;
    }
	if(req.collision==1) //object on left
	{
		const float turn_angle = 30; //turn 30 degrees (to right)
		//avoidance waypoint
        waypoint.x = req.start_x + req.collisionDistance*(cos(turn_angle*3.14159265/180)*cos(req.current_heading*3.14159265/180)-sin(turn_angle*3.14159265/180)*sin(req.current_heading*3.14159265/180)); //turn 30 deg drive 4 meters
        waypoint.y = req.start_y + req.collisionDistance*(cos(turn_angle*3.14159265/180)*sin(req.current_heading*3.14159265/180)+sin(turn_angle*3.14159265/180)*cos(req.current_heading*3.14159265/180)); //turn 30 deg drive 4 meters
        waypoint.sampleProb = 0.0;
		waypoint.terrainHazard = 0.0;
        res.waypointArrayOut.resize(1);
        res.waypointArrayOut.at(0) = waypoint;
	}
	else if(req.collision==2) //object on right
	{
		const float turn_angle = -30; //turn 30 degrees (to left)
		//avoidance waypoint
        waypoint.x = req.start_x + req.collisionDistance*(cos(turn_angle*3.14159265/180)*cos(req.current_heading*3.14159265/180)-sin(turn_angle*3.14159265/180)*sin(req.current_heading*3.14159265/180)); //turn 30 deg drive 4 meters
        waypoint.y = req.start_y + req.collisionDistance*(cos(turn_angle*3.14159265/180)*sin(req.current_heading*3.14159265/180)+sin(turn_angle*3.14159265/180)*cos(req.current_heading*3.14159265/180)); //turn 30 deg drive 4 meters
        waypoint.sampleProb = 0.0;
		waypoint.terrainHazard = 0.0;
        res.waypointArrayOut.resize(1);
        res.waypointArrayOut.at(0) = waypoint;
	}
    else if(req.collision==3) // Predictive avoidance
    {
        /*waypoint.x = req.start_x;
        waypoint.y = req.start_y;
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
        waypoint.x = req.start_x;
        waypoint.y = req.start_y;
        req.waypointArrayIn.insert(req.waypointArrayIn.begin(),waypoint);
        res.waypointArrayOut = req.waypointArrayIn;
        origNumWaypointsIn = req.waypointArrayIn.size();
        if(origNumWaypointsIn>0)
        {
            numInsertedWaypoints = 0;
            if(globalMapFullClient.call(globalMapFullSrv)) ROS_DEBUG("globalMapFull service call successful");
            else ROS_ERROR("globalMapFull service call unsuccessful");
            grid_map::GridMapRosConverter::fromMessage(globalMapFullSrv.response.globalMap, globalMap);
            mapDimensions = globalMap.getLength();
            timeOfArrivalMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
            initialViscosityMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
            resistanceMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
            waypointsToInsert.clear();
            optimalPath.clear();
            ROS_INFO("before for loop");
            for(int i=0; i<(origNumWaypointsIn-1); i++)
            {
                startPosition[0] = req.waypointArrayIn.at(i).x;
                startPosition[1] = req.waypointArrayIn.at(i).y;
                goalPoint[0] = req.waypointArrayIn.at(i+1).x;
                goalPoint[1] = req.waypointArrayIn.at(i+1).y;
                initialStraightLineCondition = straightLineDriveable(globalMap, layerToString(_satDriveability), startPosition, goalPoint, straightLineCheckHazardThresh, straightLineNumCellsOverThreshLimit, true);
                if(initialStraightLineCondition == _tooManyObstacles)
                {
                    ROS_INFO("too many obstacles along straight line, running FMM");
                    voiceSay.call("performing fast marching");
                    timeOfArrivalMap.add(timeLayer, maxTimeValue);
                    timeOfArrivalMap.add(setLayer, (float)_unknown);
                    initialViscosityMap.add(timeLayer, initialTimeValue);
                    initialViscosityMap.add(setLayer, (float)_unknown);
                    //resistanceMap.add(timeLayer, maxTimeValue);
                    resistanceMap.add(timeLayer, initialTimeValue);
                    resistanceMap.add(setLayer, (float)_unknown);
                    /*ROS_INFO("before resistanceMap FMM");
                    firsttime = ros::Time::now().toSec();
                    FMM(initialViscosityMap, resistanceMap, goalPoint);
                    nexttime = ros::Time::now().toSec();
                    deltatime = nexttime - firsttime;
                    firsttime = nexttime;
                    ROS_INFO("resistance map FMM deltaT = %lf",deltatime);*/
                    ROS_INFO("before write sat map into resistance map");
                    // resistanceMap + satMap
                    for(grid_map::GridMapIterator it(resistanceMap); !it.isPastEnd(); ++it)
                    {
                        resistanceMap.at(timeLayer, *it) = globalMap.at(layerToString(_satDriveability), *it);
                        /*globalMapValue = globalMap.at(layerToString(_satDriveability), *it);
                        if(globalMapValue >= maxTimeValue) resistanceMap.at(timeLayer, *it) += globalMapValue; // Just add or average?
                        else if(globalMapValue < maxTimeValue && globalMapValue > 0.0) resistanceMap.at(timeLayer, *it) += globalMapValueScaleFactor*globalMapValue; // Just add or average?
                        if(resistanceMap.at(timeLayer, *it) > maxTimeValue) resistanceMap.at(timeLayer, *it) = maxTimeValue;*/
                    }
                    ROS_INFO("before timeOfArrivalMap FMM");
                    firsttime = ros::Time::now().toSec();
                    FMM(resistanceMap, timeOfArrivalMap, goalPoint);
                    nexttime = ros::Time::now().toSec();
                    deltatime = nexttime - firsttime;
                    firsttime = nexttime;
                    ROS_INFO("time of arrival map FMM deltaT = %lf",deltatime);
                    //ROS_INFO("before gradientDescent");
                    //ROS_INFO("optimalPath.size() = %u",optimalPath.size());
                    gradientDescent(timeOfArrivalMap, startPosition, optimalPath);
                    if(optimalPath.size()>1)
                    {
                        //ROS_INFO("before chooseWaypointsFromOptimalPath");
                        //ROS_INFO("optimalPath.size() = %u",optimalPath.size());
                        chooseWaypointsFromOptimalPath();
                        //ROS_INFO("before waypointsOut.insert");
                        //ROS_INFO("optimalPath.size() = %u",optimalPath.size());
                    }
                    if(waypointsToInsert.size()>0)
                    {
                        res.waypointArrayOut.insert(res.waypointArrayOut.begin()+1+i+numInsertedWaypoints, waypointsToInsert.begin(), waypointsToInsert.end());
                        numInsertedWaypoints += waypointsToInsert.size();
                    }
                }
                else if(initialStraightLineCondition==_distanceTooLong)
                {
                    ROS_INFO("straight line distance too long");
                    stillInsertingWaypoints = true;
                    angleBetweenStartAndGoal = atan2(goalPoint[1]-startPosition[1], goalPoint[0]-startPosition[0]); // rad
                    workingPos = startPosition;
                    while(stillInsertingWaypoints)
                    {
                        insertWaypoint.x = maxDriveDistance*cos(angleBetweenStartAndGoal) + workingPos[0];
                        insertWaypoint.y = maxDriveDistance*sin(angleBetweenStartAndGoal) + workingPos[1];
                        res.waypointArrayOut.insert(res.waypointArrayOut.begin()+1+i+numInsertedWaypoints, insertWaypoint);
                        numInsertedWaypoints++;
                        if(hypot(goalPoint[0]-insertWaypoint.x, goalPoint[1]-insertWaypoint.y) < maxDriveDistance) stillInsertingWaypoints = false;
                        else
                        {
                            stillInsertingWaypoints = true;
                            workingPos[0] = insertWaypoint.x;
                            workingPos[1] = insertWaypoint.y;
                        }
                    }
                }
            }
            transitionWaypoints(res.waypointArrayOut);
            // ------------------------------------
            //ROS_INFO("before generateAngPubVizMap");
            generateAndPubVizMap(res.waypointArrayOut);
            // ------------------------------------
            //ROS_INFO("before waypointsOut.erase(begin)");
            res.waypointArrayOut.erase(res.waypointArrayOut.begin()); // Do not send current location as a waypoint to travel
        }
        else ROS_ERROR("called intermediate waypoints with no waypointsIn");

        /*waypoint.x = req.start_x;
        waypoint.y = req.start_y;
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
/*
    double firsttime;
    double nexttime;
    double deltatime;
*/
    float delta;
    float dx;
    float dy;
    bool continueLoop = true;
    float bestNarrowBandValue;
    float oldValue;
    float newValue;
    grid_map::Index bestNarrowBandIndex;
    grid_map::Size mapSize;
    grid_map::Index currentIndex;
    grid_map::Index offsetIndex;
    unsigned long int counter = 0;
    //mapOut.add(timeLayer, 10.0);
    mapOut.add(setLayer, (float)_unknown);
    mapSize = mapOut.getSize();
    mapOut.getIndex(goalPointIn, currentIndex);
    // Initialize narrowBandSet
    //firsttime = ros::Time::now().toSec();
    narrowBandSet.clear();
    mapCell.value = 0.0;
    mapCell.mapIndex = currentIndex;
    addToSet(narrowBandSet, mapCell);
    mapOut.atPosition(timeLayer, goalPointIn) = 0.0;
    mapOut.atPosition(setLayer, goalPointIn) = (float)_narrowBand;
    //ROS_INFO("mapSize = (%i,%i)",mapSize[0],mapSize[1]);
    /*
    nexttime = ros::Time::now().toSec();
    deltatime = nexttime - firsttime;
    firsttime = nexttime;
    ROS_INFO("FMM=> pre-loop time = %lf",deltatime);
    */
    while(continueLoop)
    {
        //ROS_INFO("narrowBandSet.size = %u", narrowBandSet.size());
        /*
        nexttime = ros::Time::now().toSec();
        deltatime = nexttime - firsttime;
        firsttime = nexttime;
        ROS_INFO("FMM=> time to top of loop = %lf",deltatime);
        */
        //ROS_INFO("counter = %u",counter);
        counter++;
        continueLoop = narrowBandNotEmpty()/* || currentIndex != startIndex*/;
        //ROS_INFO("continueLoop = %i",continueLoop);
        if(continueLoop)
        {
            bestNarrowBandValue = maxTimeValue+initialTimeValue;
            /*for(grid_map::GridMapIterator it(mapOut); !it.isPastEnd(); ++it)
            {
                if(mapOut.at(setLayer, *it) == (float)_narrowBand)
                {
                    if(mapOut.at(timeLayer, *it) < bestNarrowBandValue)
                    {
                        bestNarrowBandValue = mapOut.at(timeLayer, *it);
                        bestNarrowBandIndex = grid_map::getIndexFromLinearIndex(it.getLinearIndex(), mapOut.getSize());
                    }
                }
            }*/
            bestNarrowBandValue = narrowBandSet.begin()->value;
            bestNarrowBandIndex = narrowBandSet.begin()->mapIndex;
            mapOut.at(setLayer, bestNarrowBandIndex) = (float)_frozen;
            /*
            nexttime = ros::Time::now().toSec();
            deltatime = nexttime - firsttime;
            firsttime = nexttime;
            ROS_INFO("FMM=> top of loop to pre remove from narrow band = %lf",deltatime);
            */
            removeFromSet(narrowBandSet, bestNarrowBandValue, bestNarrowBandIndex);
            /*
            nexttime = ros::Time::now().toSec();
            deltatime = nexttime - firsttime;
            firsttime = nexttime;
            ROS_INFO("FMM=> removeFromSet NB -> Froz time = %lf",deltatime);
            */
            currentIndex = bestNarrowBandIndex;
            for(int i=-1; i<=1; i++)
            {
                for(int j=-1; j<=1; j++)
                {
                    /*
                    nexttime = ros::Time::now().toSec();
                    deltatime = nexttime - firsttime;
                    firsttime = nexttime;
                    ROS_INFO("FMM=> time to top adjacent loops = %lf",deltatime);
                    */
                    //ROS_INFO("i = %i, j = %i",i,j);
                    if((i != 0 && j == 0) || (i == 0 && j != 0))
                    {
                        offsetIndex[0] = currentIndex[0] + i;
                        offsetIndex[1] = currentIndex[1] + j;
                        if((offsetIndex[0]<mapSize[0] && offsetIndex[0]>=0) && (offsetIndex[1]<mapSize[1] && offsetIndex[1]>=0))
                        {
                            //ROS_INFO("mapSize = (%i,%i)",mapSize[0],mapSize[1]);
                            //ROS_INFO("currentIndex = (%i,%i); offsetIndex = (%i,%i)",currentIndex[0],currentIndex[1],offsetIndex[0],offsetIndex[1]);
                            /*ROS_INFO("offset neighbors = (%i,%i), (%i,%i), (%i,%i), (%i,%i)", offsetIndex[0]-1, offsetIndex[1], offsetIndex[0]+1, offsetIndex[1],
                                    offsetIndex[0], offsetIndex[1]-1, offsetIndex[0], offsetIndex[1]+1);*/
                            if(mapOut.at(setLayer, offsetIndex)!=(float)_frozen)
                            {
                                /*
                                nexttime = ros::Time::now().toSec();
                                deltatime = nexttime - firsttime;
                                firsttime = nexttime;
                                ROS_INFO("FMM=> adjcnt loop if's time = %lf",deltatime);
                                */
                                if(mapOut.at(setLayer, offsetIndex)==(float)_unknown)
                                {
                                    mapOut.at(setLayer, offsetIndex) = (float)_narrowBand;
                                    mapCell.value = mapOut.at(timeLayer, offsetIndex);
                                    mapCell.mapIndex = offsetIndex;
                                    addToSet(narrowBandSet, mapCell);
                                    //swapCellInSet(unknownSet, narrowBandSet, mapOut.at(timeLayer, offsetIndex), offsetIndex);
                                }
                                /*
                                nexttime = ros::Time::now().toSec();
                                deltatime = nexttime - firsttime;
                                firsttime = nexttime;
                                ROS_INFO("FMM=> addToSet -> NB = %lf",deltatime);
                                */

                                if(offsetIndex[0]-1 < 0 || offsetIndex[0]+1 >= mapSize[0]) dx = maxTimeValue;
                                else
                                {
                                    if(mapOut.get(timeLayer)(offsetIndex[0]-1,offsetIndex[1]) < mapOut.get(timeLayer)(offsetIndex[0]+1,offsetIndex[1])) dx = mapOut.get(timeLayer)(offsetIndex[0]-1,offsetIndex[1]);
                                    else dx = mapOut.get(timeLayer)(offsetIndex[0]+1,offsetIndex[1]);
                                }

                                if(offsetIndex[1]-1 < 0 || offsetIndex[1]+1 >= mapSize[1]) dy = maxTimeValue;
                                else
                                {
                                    if(mapOut.get(timeLayer)(offsetIndex[0],offsetIndex[1]-1) < mapOut.get(timeLayer)(offsetIndex[0],offsetIndex[1]+1)) dy = mapOut.get(timeLayer)(offsetIndex[0],offsetIndex[1]-1);
                                    else dy = mapOut.get(timeLayer)(offsetIndex[0],offsetIndex[1]+1);
                                }

                                delta = 2.0*mapIn.at(timeLayer, offsetIndex) - pow(dx-dy, 2.0);
                                //ROS_INFO("delta = %f",delta);
                                //ROS_INFO("dx = %f; dy = %f",dx,dy);
                                oldValue = mapOut.at(timeLayer, offsetIndex);
                                if(delta>=0.0)
                                {
                                    //ROS_INFO("delta>=0");
                                    newValue = (dx+dy+sqrt(delta))/2.0;
                                }
                                else
                                {
                                    //ROS_INFO("delta<0");
                                    if(dx+mapIn.at(timeLayer, offsetIndex) < dy+mapIn.at(timeLayer, offsetIndex)) newValue = dx+mapIn.at(timeLayer, offsetIndex);
                                    else newValue = dy+mapIn.at(timeLayer, offsetIndex);
                                }
                                mapOut.at(timeLayer, offsetIndex) = newValue;
                                /*
                                nexttime = ros::Time::now().toSec();
                                deltatime = nexttime - firsttime;
                                firsttime = nexttime;
                                ROS_INFO("FMM=> value computation time = %lf",deltatime);
                                */
                                modifyValueOfIndexInSet(narrowBandSet, oldValue, newValue, offsetIndex);
                                /*
                                nexttime = ros::Time::now().toSec();
                                deltatime = nexttime - firsttime;
                                firsttime = nexttime;
                                ROS_INFO("FMM=> Modify value time = %lf",deltatime);
                                */
                                if(mapOut.at(timeLayer, offsetIndex)>maxTimeValue) mapOut.at(timeLayer, offsetIndex) = maxTimeValue;
                                //else if(mapOut.at(timeLayer, offsetIndex)<minFMMTimeValue) mapOut.at(timeLayer, offsetIndex) = minFMMTimeValue;
                                //ROS_INFO("mapOut(offsetIndex) = %f",mapOut.at(timeLayer, offsetIndex));
                            }
                        }
                    }
                }
            }
            /*
            nexttime = ros::Time::now().toSec();
            deltatime = nexttime - firsttime;
            firsttime = nexttime;
            ROS_INFO("FMM=> time to bottom of adjcnt for loops = %lf",deltatime);
            */
        }
    }
}

bool SafePathing::narrowBandNotEmpty()
{
    return !narrowBandSet.empty();
}

void SafePathing::gradientDescent(grid_map::GridMap &map, grid_map::Position startPosition, std::vector<grid_map::Index> &pathOut)
{
    grid_map::Size mapSize;
    bool continueLoop = true;
    float nextBestValue = map.atPosition(timeLayer, startPosition); // Initialize nextBestValue to starting position value
    grid_map::Index currentIndex;
    grid_map::Index offsetIndex;
    grid_map::Index nextBestIndex;
    unsigned int counter = 0;
    float candidateValue;
    map.getIndex(startPosition, currentIndex);
    mapSize = map.getSize();
    int i, j;
    while(continueLoop)
    {
        //ROS_INFO("counter = %u",counter);
        //ROS_INFO("currentIndex = (%i,%i)",currentIndex[0],currentIndex[1]);
        counter++;
        continueLoop = false;
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
                        candidateValue = map.at(timeLayer, offsetIndex);
                        //ROS_INFO("candidateValue(%i,%i) = %f",i,j,candidateValue);
                        if(candidateValue < nextBestValue)
                        {
                            nextBestValue = candidateValue;
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
    float distanceToStartIndex;
    float hazardAlongPossiblePathThresh;
    bool continueLoop = true;
    robot_control::Waypoint waypointToPushBack;
    int optimalPathConsiderIndex = optimalPath.size()-1;
    int optimalPathStartIndex = 0;
    bool intersectingMinDistance = false;
    char temp;

    waypointsToInsert.clear();
    startIndex = optimalPath.front();
    indexToConsider = optimalPath.back();
    hazardAlongPossiblePathThresh = initialHazardAlongPossiblePathThresh;
    while(continueLoop)
    {
        timeOfArrivalMap.getPosition(startIndex, startIndexPos);
        timeOfArrivalMap.getPosition(indexToConsider, considerIndexPos);
        //ROS_INFO("################################################");
        //ROS_INFO("startIndex = (%i,%i)",startIndex[0],startIndex[1]);
        //ROS_INFO("indexToConsider = (%i,%i)",indexToConsider[0],indexToConsider[1]);
        /*if(hypot(considerIndexPos[0]-startIndexPos[0], considerIndexPos[1]-startIndexPos[1]) < minWaypointDistance)
        {
            continueLoop = false;
            break;
        }*/
        //ROS_INFO("hazardAlongPossiblePathThresh = %f",hazardAlongPossiblePathThresh);
        if(straightLineDriveable(resistanceMap, timeLayer, startIndexPos, considerIndexPos, hazardAlongPossiblePathThresh, numCellsOverThreshLimit, false)!=_driveable && !intersectingMinDistance)
        {
            //ROS_INFO("========= numHazardsOverLimit");
            prevIndex = indexToConsider;
            //ROS_INFO("optimalPathConsiderIndex = %i",optimalPathConsiderIndex);
            //ROS_INFO("optimalPathStartIndex = %i",optimalPathStartIndex);
            for(optimalPathConsiderIndex; optimalPathConsiderIndex>=optimalPathStartIndex; optimalPathConsiderIndex--)
            {
                //ROS_INFO("optimalPath.size = %u",optimalPath.size());
                //ROS_INFO("optimalPathStartIndex = %i",optimalPathStartIndex);
                //ROS_INFO("optimalPathConsiderIndex = %i",optimalPathConsiderIndex);
                indexToConsider = optimalPath.at(optimalPathConsiderIndex);
                timeOfArrivalMap.getPosition(indexToConsider, considerIndexPos);
                timeOfArrivalMap.getPosition(prevIndex, prevIndexPos);
                //ROS_INFO("indexToConsider = (%i,%i)",indexToConsider[0],indexToConsider[1]);
                //ROS_INFO("considerIndexPos = (%f,%f)",considerIndexPos[0],considerIndexPos[1]);
                //ROS_INFO("prevIndexPos = (%f,%f)",prevIndexPos[0],prevIndexPos[1]);
                distanceBetweenIndices = hypot(considerIndexPos[0]-prevIndexPos[0], considerIndexPos[1]-prevIndexPos[1]);
                //ROS_INFO("distanceBetween = %f",distanceBetweenIndices);
                distanceToStartIndex = hypot(considerIndexPos[0]-startIndexPos[0], considerIndexPos[1]-startIndexPos[1]);
                //ROS_INFO("distanceToStartIndex = %f",distanceToStartIndex);
                //ROS_INFO("optimalPath.back = (%i,%i", optimalPath.back()[0], optimalPath.back()[1]);
                //ROS_INFO("indexToConsider = (%i,%i)",indexToConsider[0], indexToConsider[1]);
                if(distanceToStartIndex<=minWaypointDistance && distanceBetweenIndices<=minWaypointDistance)
                {
                    //ROS_INFO("%%%% intersecting min distance");
                    intersectingMinDistance = true;
                }
                if(distanceToStartIndex<minWaypointDistance)
                {
                    //ROS_INFO("!*!* distance to start index < min");
                    hazardAlongPossiblePathThresh += hazardThreshIncrementAmount;
                    if(!intersectingMinDistance)
                    {
                        optimalPathConsiderIndex = optimalPath.size()-1;
                        indexToConsider = optimalPath.at(optimalPathConsiderIndex);
                    }
                    break;
                }
                else if(distanceBetweenIndices>minWaypointDistance)
                {
                    //ROS_INFO("distacenBetweenIndices > min");
                    break;
                }
                /*if(optimalPathConsiderIndex == optimalPathStartIndex)
                {
                    std::cout << "enter a character to continue" << std::endl;
                    std::cin >> temp;
                    if(temp == 'q') exit(1);
                }*/
            }
            continueLoop = true;
        }
        else if(indexToConsider[0] == optimalPath.back()[0] && indexToConsider[1] == optimalPath.back()[1]) continueLoop = false; // Straight line path to end point is clear, do not insert any waypoints
        else
        {
            //ROS_INFO("push back waypoint to insert");
            waypointToPushBack.x = considerIndexPos[0];
            waypointToPushBack.y = considerIndexPos[1];
            waypointsToInsert.push_back(waypointToPushBack);
            //if(indexToConsider[0]==optimalPath.back()[0] && indexToConsider[1]==optimalPath.back()[1]) continueLoop = false;
            startIndex = indexToConsider;
            optimalPathStartIndex = optimalPathConsiderIndex;
            indexToConsider = optimalPath.back();
            optimalPathConsiderIndex = optimalPath.size()-1;
            intersectingMinDistance = false;
            continueLoop = true;
        }
        std::printf("\n");
    }
}

STRAIGHT_LINE_CONDITION_T SafePathing::straightLineDriveable(grid_map::GridMap &map, std::string layer, grid_map::Position &startPos, grid_map::Position &endPos, float hazardThresh, unsigned int numCellsLimit, bool useMinDistanceLimit)
{
    unsigned int numCellsOverThresh;
    grid_map::Polygon mapPolygon;
    std::vector<grid_map::Position> polygonVertices;
    float mapPolygonHeading;

    if(useMinDistanceLimit)
    {
        if(hypot(endPos[0] - startPos[0], endPos[1] - startPos[1]) <= minDriveDistanceForFMM) {ROS_INFO("distance too short, no need to run FMM"); return _driveable;}
    }
    numCellsOverThresh = 0;
    polygonVertices.resize(4);
    mapPolygon.removeVertices();
    mapPolygonHeading = atan2(endPos[1] - startPos[1], endPos[0] - startPos[0]); // radians
    //ROS_INFO("create verticies");
    polygonVertices.at(0)[0] = startPos[0] + corridorWidth/2.0*sin(mapPolygonHeading);
    polygonVertices.at(0)[1] = startPos[1] - corridorWidth/2.0*cos(mapPolygonHeading);
    polygonVertices.at(1)[0] = startPos[0] - corridorWidth/2.0*sin(mapPolygonHeading);
    polygonVertices.at(1)[1] = startPos[1] + corridorWidth/2.0*cos(mapPolygonHeading);
    polygonVertices.at(2)[0] = endPos[0] - corridorWidth/2.0*sin(mapPolygonHeading);
    polygonVertices.at(2)[1] = endPos[1] + corridorWidth/2.0*cos(mapPolygonHeading);
    polygonVertices.at(3)[0] = endPos[0] + corridorWidth/2.0*sin(mapPolygonHeading);
    polygonVertices.at(3)[1] = endPos[1] - corridorWidth/2.0*cos(mapPolygonHeading);
    //ROS_INFO("before addVertex");
    mapPolygon.addVertex(polygonVertices.at(0));
    mapPolygon.addVertex(polygonVertices.at(1));
    mapPolygon.addVertex(polygonVertices.at(2));
    mapPolygon.addVertex(polygonVertices.at(3));
    //ROS_INFO("after addVertex");
    for(grid_map::PolygonIterator it(map, mapPolygon); !it.isPastEnd(); ++it)
    {
        if(map.at(layer, *it) >= hazardThresh) numCellsOverThresh++;
    }
    //ROS_INFO("startPos = (%f,%f), endPos = (%f,%f)", startPos[0], startPos[1], endPos[0], endPos[1]);
    //ROS_INFO("hazardThresh = %f",hazardThresh);
    //ROS_INFO("numCellsLimit = %u",numCellsLimit);
    //ROS_INFO("numCellsOverThresh = %u",numCellsOverThresh);
    //ROS_INFO("after polygon iterator loop");
    if(numCellsOverThresh>=numCellsLimit) {ROS_INFO("too many obstacles"); return _tooManyObstacles;}
    else
    {
        if(hypot(endPos[0]-startPos[0], endPos[1]-startPos[1]) > maxDriveDistance) {ROS_INFO("distance to long"); return _distanceTooLong;}
        else {ROS_INFO("driveable"); return _driveable;}
    }
}

void SafePathing::transitionWaypoints(std::vector<robot_control::Waypoint>& waypointList)
{
    unsigned int waypointsOutInitSize;
    unsigned int numTransitionWaypointsInserted = 0;
    float firstWaypointDistance;
    float secondWaypointDistance;
    float distanceBetweenWaypoints;
    float angleBetweenWaypoints; // radians
    float robotCurrentRadius;
    bool distanceFromLineToOriginLessThanHomingRadius;
    bool angleBetweenWaypointsGreaterThan90;
    bool oneHomingWaypointSelected;
    float A;
    float B;
    float C;
    float m;
    float dx;
    float dy;
    const float minDX = 0.001; // m
    float xSol[2];
    float ySol[2];
    robot_control::Waypoint homingWaypoint;
    robot_control::Waypoint lastWaypointBeforeHome;
    robot_control::Waypoint firstWaypointOutFromHome;
    robot_control::Waypoint waypoint1;
    robot_control::Waypoint waypoint2;

    waypointsOutInitSize = waypointList.size();
    if(waypointsOutInitSize>1)
    {
        // Corner waypoint check
        robotCurrentRadius = hypot(waypointList.front().x, waypointList.front().y);
        //ROS_INFO("robotCurrentRadius = %f",robotCurrentRadius);
        if(waypointList.back().x == homeWaypoint.x && waypointList.back().y == homeWaypoint.y && robotCurrentRadius > atHomeRadius)
        {
            ROS_INFO("coming home, insert corner tansition waypoints");
            lastWaypointBeforeHome.x = waypointList.at(waypointList.size()-2).x;
            lastWaypointBeforeHome.y = waypointList.at(waypointList.size()-2).y;
            if(lastWaypointBeforeHome.x>0.0 && lastWaypointBeforeHome.y>0.0)
            {
                ROS_INFO("quad 1");
                waypointList.insert(waypointList.end()-1, quad1MagneticWaypoint);
                numTransitionWaypointsInserted++;
            }
            else if(lastWaypointBeforeHome.x<0.0 && lastWaypointBeforeHome.y>0.0)
            {
                ROS_INFO("quad 2");
                waypointList.insert(waypointList.end()-1, quad2MagneticWaypoint);
                waypointList.insert(waypointList.end()-1, quad1MagneticWaypoint);
                numTransitionWaypointsInserted+=2;
            }
            else if(lastWaypointBeforeHome.x<0.0 && lastWaypointBeforeHome.y<0.0)
            {
                ROS_INFO("quad 3");
                waypointList.insert(waypointList.end()-1, quad3MagneticWaypoint);
                waypointList.insert(waypointList.end()-1, quad4MagneticWaypoint);
                numTransitionWaypointsInserted+=2;
            }
            else
            {
                ROS_INFO("quad 4");
                waypointList.insert(waypointList.end()-1, quad4MagneticWaypoint);
                numTransitionWaypointsInserted++;
            }
        }
        else if(robotCurrentRadius <= atHomeRadius && hypot(waypointList.at(1).x, waypointList.at(1).y) > atHomeRadius)
        {
            ROS_INFO("leaving home, insert corner tansition waypoints");
            firstWaypointOutFromHome.x = waypointList.at(1).x;
            firstWaypointOutFromHome.y = waypointList.at(1).y;
            if(firstWaypointOutFromHome.x>0.0 && firstWaypointOutFromHome.y>0.0)
            {
                ROS_INFO("quad 1");
                waypointList.insert(waypointList.begin()+1, quad1MagneticWaypoint);
                numTransitionWaypointsInserted++;
            }
            else if(firstWaypointOutFromHome.x<0.0 && firstWaypointOutFromHome.y>0.0)
            {
                ROS_INFO("quad 2");
                waypointList.insert(waypointList.begin()+1, quad2MagneticWaypoint);
                waypointList.insert(waypointList.begin()+1, quad1MagneticWaypoint);
                numTransitionWaypointsInserted+=2;
            }
            else if(firstWaypointOutFromHome.x<0.0 && firstWaypointOutFromHome.y<0.0)
            {
                ROS_INFO("quad 3");
                waypointList.insert(waypointList.begin()+1, quad3MagneticWaypoint);
                waypointList.insert(waypointList.begin()+1, quad4MagneticWaypoint);
                numTransitionWaypointsInserted+=2;
            }
            else
            {
                ROS_INFO("quad 4");
                waypointList.insert(waypointList.begin()+1, quad4MagneticWaypoint);
                numTransitionWaypointsInserted++;
            }
        }
        // Homing waypoint check
        oneHomingWaypointSelected = false;
        waypointsOutInitSize = waypointList.size();
        for(int i=0; i<(waypointsOutInitSize-1); i++)
        {
            waypoint1 = waypointList.at(i);
            waypoint2 = waypointList.at(i+1);
            ROS_INFO("+++ i = %i",i);
            ROS_INFO("first waypoint = (%f,%f)",waypoint1.x,waypoint1.y);
            ROS_INFO("second waypoint = (%f,%f)",waypoint2.x,waypoint2.y);
            firstWaypointDistance = hypot(waypoint1.x, waypoint1.y);
            secondWaypointDistance = hypot(waypoint2.x, waypoint2.y);
            distanceBetweenWaypoints = hypot(waypoint1.x-waypoint2.x, waypoint1.y-waypoint2.y);
            if((fabs(waypoint1.x*waypoint2.y - waypoint1.y*waypoint2.x)/distanceBetweenWaypoints) < homingRadius) distanceFromLineToOriginLessThanHomingRadius = true;
            else distanceFromLineToOriginLessThanHomingRadius = false;
            angleBetweenWaypoints = acos((pow(firstWaypointDistance, 2.0) + pow(secondWaypointDistance, 2.0) - pow(distanceBetweenWaypoints, 2.0))/
                                         (2.0*firstWaypointDistance*secondWaypointDistance)); // radians
            if(angleBetweenWaypoints > PI/2.0) angleBetweenWaypointsGreaterThan90 = true;
            else angleBetweenWaypointsGreaterThan90 = false;
            //ROS_INFO("firstWaypointDistance = %f",firstWaypointDistance);
            //ROS_INFO("secondWaypointDistance = %f",secondWaypointDistance);
            //ROS_INFO("distanceBetweenWaypoints = %f",distanceBetweenWaypoints);
            if(((firstWaypointDistance > homingRadius && secondWaypointDistance < homingRadius) ||
                    (distanceFromLineToOriginLessThanHomingRadius && firstWaypointDistance > homingRadius && secondWaypointDistance > homingRadius
                     && angleBetweenWaypointsGreaterThan90)) && !oneHomingWaypointSelected)
            {
                ROS_INFO("stop at homing radius");
                oneHomingWaypointSelected = true;
                dy = waypoint1.y - waypoint2.y;
                dx = waypoint1.x - waypoint2.x;
                if(dx>=0.0 && fabs(dx) < minDX) dx = minDX;
                else if(dx<0.0 && fabs(dx) < minDX) dx = -minDX;
                m = dy/dx;
                A = 1.0 + pow(m, 2.0);
                B = -2.0*pow(m, 2.0)*waypoint2.x + 2.0*m*waypoint2.y;
                C = pow(m, 2.0)*pow(waypoint2.x, 2.0) - 2.0*m*waypoint2.x*waypoint2.y + pow(waypoint2.y, 2.0) - pow(homingRadius, 2.0);
                //ROS_INFO("m = %f",m);
                //ROS_INFO("A = %f",A);
                //ROS_INFO("B = %f",B);
                //ROS_INFO("C = %f",C);
                xSol[0] = (-B+sqrt(pow(B, 2.0) - 4.0*A*C))/(2.0*A);
                xSol[1] = (-B-sqrt(pow(B, 2.0) - 4.0*A*C))/(2.0*A);
                ySol[0] = waypoint2.y + m*(xSol[0] - waypoint2.x);
                ySol[1] = waypoint2.y + m*(xSol[1] - waypoint2.x);
                //ROS_INFO("homing solutions: (%f,%f); (%f,%f)", xSol[0], ySol[0], xSol[1], ySol[1]);
                if(hypot(xSol[1]-waypoint1.x, ySol[1]-waypoint1.y) < hypot(xSol[0]-waypoint1.x, ySol[0]-waypoint1.y))
                {
                    homingWaypoint.x = xSol[1];
                    homingWaypoint.y = ySol[1];
                    //ROS_INFO("homing solution 1 chosen");
                }
                else
                {
                    homingWaypoint.x = xSol[0];
                    homingWaypoint.y = ySol[0];
                    //ROS_INFO("homing solution 0 chosen");
                }
                waypointList.insert(waypointList.begin()+1+i+numTransitionWaypointsInserted, homingWaypoint);
                numTransitionWaypointsInserted++;
            }
        }
    }
}

void SafePathing::generateAndPubVizMap(std::vector<robot_control::Waypoint> waypointList)
{
    grid_map::Position waypointPositions;
    //ROS_INFO("before setGeometry");
    vizMap.setGeometry(mapDimensions, mapResolution, mapOrigin);
    //ROS_INFO("before add layers");
    vizMap.add(vizResistanceLayer, 0.0);
    vizMap.add(vizTimeOfArrivalLayer, 0.0);
    vizMap.add(vizOptimalPathLayer, 0.0);
    //ROS_INFO("before iterator loop");
    for(grid_map::GridMapIterator it(vizMap); !it.isPastEnd(); ++it)
    {
        vizMap.at(vizResistanceLayer, *it) = resistanceMap.at(timeLayer, *it);
        vizMap.at(vizTimeOfArrivalLayer, *it) = timeOfArrivalMap.at(timeLayer, *it);
    }
    if(optimalPath.size()>1)
    {
        //ROS_INFO("before optimalPath loop");
        //ROS_INFO("optimalPath.size() = %u",optimalPath.size());
        for(int i=0; i<optimalPath.size(); i++)
        {
            //ROS_INFO("i = %i", i);
            //ROS_INFO("optimalPath[i] = (%i,%i)",optimalPath.at(i)[0],optimalPath.at(i)[1]);
            vizMap.at(vizOptimalPathLayer, optimalPath.at(i)) = 5.0;
        }
    }
    for(int i=0; i<waypointList.size(); i++)
    {
        waypointPositions[0] = waypointList.at(i).x;
        waypointPositions[1] = waypointList.at(i).y;
        vizMap.atPosition(vizOptimalPathLayer, waypointPositions) = 10.0;
    }
    //ROS_INFO("before toMessage");
    grid_map::GridMapRosConverter::toMessage(vizMap, vizMapMsg);
    //ROS_INFO("before publish");
    vizMapPub.publish(vizMapMsg);
}

void SafePathing::addToSet(std::multiset<MapData, MapDataLess>& set, MapData& cell)
{
    set.insert(cell);
}

void SafePathing::removeFromSet(std::multiset<MapData, MapDataLess>& set, float cellValue, grid_map::Index mapIndex)
{
    /*
    double firsttime;
    double nexttime;
    double deltatime;
    unsigned long int numIterations;
    */
    std::pair<std::multiset<MapData, MapDataLess>::iterator,std::multiset<MapData, MapDataLess>::iterator> removeRange;
    std::multiset<MapData, MapDataLess>::iterator it;
    MapData testCell;
    testCell.value = cellValue;
    testCell.mapIndex = mapIndex;
    //firsttime = ros::Time::now().toSec();
    removeRange = set.equal_range(testCell);
    /*
    nexttime = ros::Time::now().toSec();
    deltatime = nexttime - firsttime;
    firsttime = nexttime;
    ROS_INFO("removeFromSet=> after set.equal_range = %lf",deltatime);
    */
    //numIterations = 0;
    it = removeRange.first;
    do
    {
        if(it->mapIndex[0] == mapIndex[0] && it->mapIndex[1] == mapIndex[1])
        {
            //ROS_INFO("!!!!! found cell to erase !!!!");
            /*
            nexttime = ros::Time::now().toSec();
            deltatime = nexttime - firsttime;
            firsttime = nexttime;
            ROS_INFO("removeFromSet=> before set.erase = %lf",deltatime);
            */
            set.erase(it);
            /*
            nexttime = ros::Time::now().toSec();
            deltatime = nexttime - firsttime;
            firsttime = nexttime;
            ROS_INFO("removeFromSet=> after set.erase = %lf",deltatime);
            */
            break;
        }
        ++it;
        //numIterations++;
    }while(it!=removeRange.second);
    //ROS_INFO("numIterations = %u",numIterations);
    /*
    nexttime = ros::Time::now().toSec();
    deltatime = nexttime - firsttime;
    firsttime = nexttime;
    ROS_INFO("removeFromSet=> after do-while loop = %lf",deltatime);
    */
}

void SafePathing::modifyValueOfIndexInSet(std::multiset<MapData, MapDataLess>& set, float oldCellValue, float newCellValue, grid_map::Index mapIndex)
{
    MapData newCell;
    newCell.value = newCellValue;
    newCell.mapIndex = mapIndex;
    removeFromSet(set, oldCellValue, mapIndex);
    addToSet(set, newCell);
}

void SafePathing::swapCellInSet(std::multiset<MapData, MapDataLess>& fromSet, std::multiset<MapData, MapDataLess>& toSet, float cellValue, grid_map::Index mapIndex)
{
    //***
    double firsttime;
    double nexttime;
    double deltatime;
    //***

    MapData swapCell;
    swapCell.value = cellValue;
    swapCell.mapIndex = mapIndex;
    //***
    firsttime = ros::Time::now().toSec();
    //***
    removeFromSet(fromSet, cellValue, mapIndex);
    //***
    nexttime = ros::Time::now().toSec();
    deltatime = nexttime - firsttime;
    firsttime = nexttime;
    ROS_INFO("swapCellInSet=> after removeFromSet = %lf",deltatime);
    //***
    addToSet(toSet, swapCell);
    //***
    nexttime = ros::Time::now().toSec();
    deltatime = nexttime - firsttime;
    firsttime = nexttime;
    ROS_INFO("swapCellInSet=> after addToSet = %lf",deltatime);
    //***
}
