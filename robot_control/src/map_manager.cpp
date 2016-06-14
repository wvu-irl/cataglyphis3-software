#include <robot_control/map_manager.h>

MapManager::MapManager()
{
    roiServ = nh.advertiseService("/control/mapmanager/regionsofinterest", &MapManager::listROI, this);
    modROIServ = nh.advertiseService("/control/mapmanager/modifyroi", &MapManager::modROI, this);
    searchMapServ = nh.advertiseService("/control/mapmanager/searchlocalmap", &MapManager::searchMapCallback, this);
    globalMapPathHazardsServ = nh.advertiseService("/control/mapmanager/globalmappathhazards", &MapManager::globalMapPathHazardsCallback, this);
    searchLocalMapInfoServ = nh.advertiseService("/control/mapmanager/searchlocalmapinfo", &MapManager::searchLocalMapInfoCallback, this);
    createROIKeyframeClient = nh.serviceClient<messages::CreateROIKeyframe>("/slam/keyframesnode/createroikeyframe");
    keyframesSub = nh.subscribe<messages::KeyframeList>("/slam/keyframesnode/keyframelist", 1, &MapManager::keyframesCallback, this);
    globalPoseSub = nh.subscribe<messages::RobotPose>("/hsm/masterexec/globalpose", 1, &MapManager::globalPoseCallback, this);
    keyframeRelPoseSub = nh.subscribe<messages::SLAMPoseOut>("/slam/keyframesnode/slamposeout", 1, &MapManager::keyframeRelPoseCallback, this);
    currentROIPub = nh.advertise<robot_control::CurrentROI>("/control/mapmanager/currentroi", 1);
    globalMapPub = nh.advertise<grid_map_msgs::GridMap>("/control/mapmanager/globalmapviz", 1);
    searchLocalMapPub = nh.advertise<grid_map_msgs::GridMap>("/control/mapmanager/searchlocalmapviz", 1);
    currentROIMsg.currentROINum = 0; // 0 means in no ROI
    searchLocalMapROINum = 0;
    keyframeWriteIntoGlobalMapSerialNum = 0;
    searchLocalMapExists = false;
    globalMapPathHazardsVertices.resize(4);
    globalPose.northAngle = 90.0; // degrees. initial guess
    previousNorthAngle = 0.0; // degrees. different than actual north angle to force update first time through

    // Temporary ROIs. Rectangle around starting platform
    ROI.e = 8.0;
    ROI.s = 5.0;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.e = -8.0;
    ROI.s = 5.0;
    ROI.purpleProb = 500;
    ROI.redProb = 500;
    ROI.blueProb = 500;
    ROI.silverProb = 500;
    ROI.brassProb = 500;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.e = -8.0;
    ROI.s = -5.0;
    ROI.purpleProb = 400;
    ROI.redProb = 400;
    ROI.blueProb = 400;
    ROI.silverProb = 400;
    ROI.brassProb = 400;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.e = 8.0;
    ROI.s = -5.0;
    ROI.purpleProb = 300;
    ROI.redProb = 300;
    ROI.blueProb = 300;
    ROI.silverProb = 300;
    ROI.brassProb = 300;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    northTransformROIs();

    // Temporary ROIs. Search in front of library
    /*ROI.e = 35.0826;
    ROI.s = 20.9706;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.e = 34.9990;
    ROI.s = 28.0289;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.e = 42.0208;
    ROI.s = 28.1840;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.e = 48.9591;
    ROI.s = 28.0289;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.e = 48.9591;
    ROI.s = 21.0482;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.e = 42.1044;
    ROI.s = 21.2033;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.e = 41.9373;
    ROI.s = 34.9320;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.e = 45.7825;
    ROI.s = 31.5968;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.e = 45.4482;
    ROI.s = 24.3059;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.e = 38.5099;
    ROI.s = 24.5385;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.e = 38.5935;
    ROI.s = 31.5968;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.radialAxis = 20.0;
    ROI.tangentialAxis = 15.0;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    northTransformROIs();*/

	// ***********************************
    /*globalMapPub = nh.advertise<grid_map_msgs::GridMap>("control/mapmanager/globalmap",1);
    globalMap.setFrameId("map");
    globalMap.setGeometry(grid_map::Length(300.0, 200.0), mapResolution, grid_map::Position(12.0, 65.0));
    globalMap.add("sampleProb", 458);
    globalMap.add("hazard", 998);
    //globalMap.
    ROS_INFO("globalMap x len = %i",globalMap.getSize()(0));
    ROS_INFO("globalMap y len = %i",globalMap.getSize()(1));
    ROS_INFO("globalMap resolution = %f",globalMap.getResolution());
    ROS_INFO("globalMap x lenF = %f",(float)globalMap.getSize()(0)*globalMap.getResolution());
    ROS_INFO("globalMap y lenF = %f",(float)globalMap.getSize()(1)*globalMap.getResolution());
    for(grid_map::GridMapIterator it(globalMap); !it.isPastEnd(); ++it)
    {
        globalMap.at("sampleProb", *it) = 322.9;
        globalMap.at("hazard", *it) = 190.1;
    }
    globalMap.atPosition("sampleProb", grid_map::Position(50.1, 7.0)) = 500.2;
    ROS_INFO("prob at (80,80) = %f", globalMap.atPosition("sampleProb", grid_map::Position(80.0, 80.0)));
    ROS_INFO("prob at (90,7.2) = %f", globalMap.atPosition("sampleProb", grid_map::Position(90.0, 7.2)));
    ROS_INFO("prob at (90.1,7.0) = %f", globalMap.atPosition("sampleProb", grid_map::Position(90.1, 7.0)));
    ROS_INFO("prob at (90.5,7.0) = %f", globalMap.atPosition("sampleProb", grid_map::Position(90.5, 7.0)));
    ROS_INFO("prob at (90.5,6.8) = %f", globalMap.atPosition("sampleProb", grid_map::Position(90.5, 6.8)));
    grid_map::GridMapRosConverter::toMessage(globalMap,globalMapMsg);
    ros::Duration(1.0).sleep();
    globalMapPub.publish(globalMapMsg);*/

    ros::Duration(1).sleep();

    globalMap.setFrameId("map");
    globalMapTemp.setFrameId("map");
#ifdef EVANSDALE
    satMapSize[0] = 200.0;
    satMapSize[1] = 100.0;
    //globalMapOrigin[0] = 66.667;
    //globalMapOrigin[1] = 17.143;
    globalMapOrigin[0] = 0.0;
    globalMapOrigin[1] = 0.0;
#endif // EVANSDALE
#ifdef INSTITUTE_PARK
    satMapSize[0] = 400.0;
    satMapSize[1] = 300.0;
    //globalMapOrigin[0] = 12.0;
    //globalMapOrigin[1] = 65.0;
    globalMapOrigin[0] = 0.0;
    globalMapOrigin[1] = 0.0;
#endif // INSTUTUTE_PARK
    calculateGlobalMapSize();
    //globalMapSize[0] = hypot(satMapSize[0],satMapSize[1]) + std::max(fabs(globalMapOrigin[0]),fabs(globalMapOrigin[1]));
    //globalMapSize[1] = globalMapSize[0];
    globalMap.setGeometry(globalMapSize, mapResolution, globalMapOrigin);
    globalMapTemp.setGeometry(globalMapSize, mapResolution, globalMapOrigin);
    globalMapRowsCols = globalMap.getSize();
    //gridMapAddLayers(0, MAP_KEYFRAME_LAYERS_END_INDEX, globalMap);
    gridMapAddLayers(0, NUM_MAP_LAYERS-1, globalMap);
    gridMapAddLayers(0, NUM_MAP_LAYERS-1, globalMapTemp);
    writeSatMapIntoGlobalMap();
    globalMapPathHazardsPolygon.setFrameId(globalMap.getFrameId());

    searchLocalMap.setFrameId("map");
    searchLocalMap.setGeometry(grid_map::Length(searchLocalMapLength, searchLocalMapWidth), mapResolution, grid_map::Position(0.0, 0.0));
    gridMapAddLayers(0, NUM_MAP_LAYERS-1, searchLocalMap);

    currentKeyframe.setFrameId("map");
    ROIKeyframe.setFrameId("map");

    /*ROS_DEBUG("before global map toMsssage");
    grid_map::GridMapRosConverter::toMessage(globalMap, globalMapMsg);
    ROS_DEBUG("after global map toMsssage");
    globalMapPub.publish(globalMapMsg);
    ROS_DEBUG("after global map publish");*/
}

bool MapManager::listROI(robot_control::RegionsOfInterest::Request &req, robot_control::RegionsOfInterest::Response &res) // tested
{
    res.ROIList = regionsOfInterest;
    ROS_INFO("sent ROI");
    return true;
}

bool MapManager::modROI(robot_control::ModifyROI::Request &req, robot_control::ModifyROI::Response &res) // need to add more features
{
    if(req.setSearchedROI) regionsOfInterest.at(req.numSearchedROI).searched = req.searchedROIState;
    if(req.addNewROI)
    {
        regionsOfInterest.push_back(req.newROI);
    }
    ROS_INFO("modified ROI");
    return true;
}

bool MapManager::searchMapCallback(robot_control::SearchMap::Request &req, robot_control::SearchMap::Response &res) // tested
{
    if(req.createMap && !req.deleteMap)
    {
        if(searchLocalMapExists) ROS_WARN("MAP MANAGER: tried to create searchLocalMap when it already exists");
        else
        {
            createROIKeyframeSrv.request.roiIndex = req.roiIndex;
            if(createROIKeyframeClient.call(createROIKeyframeSrv)) ROS_DEBUG("MAP MANAGER: createROIKeyframe service call successful"); // call createROIKeyframe service
            else {ROS_ERROR("MAP MANAGER: createROIKeyframe service call unsuccessful"); return false;}
            searchLocalMapExists = true;
            grid_map::GridMapRosConverter::fromMessage(createROIKeyframeSrv.response.keyframe.map, ROIKeyframe);
            searchLocalMapToROIAngle = RAD2DEG*atan2(regionsOfInterest.at(req.roiIndex).s, regionsOfInterest.at(req.roiIndex).e) - fmod(createROIKeyframeSrv.response.keyframe.heading, 360.0);
            sigmaROIX = regionsOfInterest.at(req.roiIndex).radialAxis/2.0/numSigmasROIAxis;
            sigmaROIY = regionsOfInterest.at(req.roiIndex).tangentialAxis/2.0/numSigmasROIAxis;
            for(grid_map::GridMapIterator it(searchLocalMap); !it.isPastEnd(); ++it)
            {
                searchLocalMap.getPosition(*it, searchLocalMapCoord);
                ROS_INFO("searchLocalMapCoord: x = %f; y = %f",searchLocalMapCoord[0], searchLocalMapCoord[1]);
                rotateCoord(searchLocalMapCoord[0], searchLocalMapCoord[1], ROIX, ROIY, searchLocalMapToROIAngle);
                //ROIX = searchLocalMapCoord[0]*cos(DEG2RAD*searchLocalMapToROIAngle)+searchLocalMapCoord[1]*sin(DEG2RAD*searchLocalMapToROIAngle);
                //ROIY = -searchLocalMapCoord[0]*sin(DEG2RAD*searchLocalMapToROIAngle)+searchLocalMapCoord[1]*cos(DEG2RAD*searchLocalMapToROIAngle);
                ROS_INFO("ROIX = %f; ROIY = %f",ROIX, ROIY);
                for(int j=MAP_KEYFRAME_LAYERS_START_INDEX; j<=MAP_KEYFRAME_LAYERS_END_INDEX; j++)
                {
                    searchLocalMap.at(layerToString(static_cast<MAP_LAYERS_T>(j)), *it) = ROIKeyframe.atPosition(layerToString(static_cast<MAP_LAYERS_T>(j)), searchLocalMapCoord);
                }
                for(int k=MAP_SAMPLE_PROB_LAYERS_START_INDEX; k<=MAP_SAMPLE_PROB_LAYERS_END_INDEX; k++)
                {
                    // Add condition to exclude sample types already known to be completely collected
                    searchLocalMap.at(layerToString(static_cast<MAP_LAYERS_T>(k)), *it) = sampleProbPeak*exp(-(pow(ROIX,2.0)/(2.0*pow(sigmaROIX,2.0))+pow(ROIY,2.0)/(2.0*pow(sigmaROIY,2.0))));
                }
            }
            grid_map::GridMapRosConverter::toMessage(searchLocalMap, searchLocalMapMsg);
            searchLocalMapPub.publish(searchLocalMapMsg);
        }
    }
    else if(req.deleteMap && !req.createMap)
    {
        searchLocalMapExists = false;
        searchLocalMap.clearBasic();
    }
    return true;
}

bool MapManager::globalMapPathHazardsCallback(messages::GlobalMapPathHazards::Request &req, messages::GlobalMapPathHazards::Response &res) // tested
{
    globalMapPathHazardsPolygon.removeVertices();
    res.numHazards = 0;
    res.hazardList.clear();
    globalMapPathHazardsPolygonHeading = atan2(req.yEnd - req.yStart, req.xEnd - req.xStart); // radians
    globalMapPathHazardsVertices.at(0)[0] = req.xStart + req.width/2.0*sin(globalMapPathHazardsPolygonHeading);
    globalMapPathHazardsVertices.at(0)[1] = req.yStart - req.width/2.0*cos(globalMapPathHazardsPolygonHeading);
    globalMapPathHazardsVertices.at(1)[0] = req.xStart - req.width/2.0*sin(globalMapPathHazardsPolygonHeading);
    globalMapPathHazardsVertices.at(1)[1] = req.yStart + req.width/2.0*cos(globalMapPathHazardsPolygonHeading);
    globalMapPathHazardsVertices.at(2)[0] = req.xEnd - req.width/2.0*sin(globalMapPathHazardsPolygonHeading);
    globalMapPathHazardsVertices.at(2)[1] = req.yEnd + req.width/2.0*cos(globalMapPathHazardsPolygonHeading);
    globalMapPathHazardsVertices.at(3)[0] = req.xEnd + req.width/2.0*sin(globalMapPathHazardsPolygonHeading);
    globalMapPathHazardsVertices.at(3)[1] = req.yEnd - req.width/2.0*cos(globalMapPathHazardsPolygonHeading);
    globalMapPathHazardsPolygon.addVertex(globalMapPathHazardsVertices.at(0));
    globalMapPathHazardsPolygon.addVertex(globalMapPathHazardsVertices.at(1));
    globalMapPathHazardsPolygon.addVertex(globalMapPathHazardsVertices.at(2));
    globalMapPathHazardsPolygon.addVertex(globalMapPathHazardsVertices.at(3));
    for(grid_map::PolygonIterator it(globalMap, globalMapPathHazardsPolygon); !it.isPastEnd(); ++it)
    {
        globalMapPathHazardValue = globalMap.at(layerToString(_driveability), *it);
        if(globalMapPathHazardValue > 0.0)
        {
            res.numHazards++;
            globalMap.getPosition(*it, globalMapPathHazardPosition);
            res.hazardList.push_back(messages::DriveabilityHazards());
            res.hazardList.at(res.numHazards-1).type = (int)globalMapPathHazardValue;
            res.hazardList.at(res.numHazards-1).localX = globalMapPathHazardPosition[0] - req.xStart;
            res.hazardList.at(res.numHazards-1).localY = globalMapPathHazardPosition[1] - req.yStart;
            rotateCoord(res.hazardList.at(res.numHazards-1).localX,res.hazardList.at(res.numHazards-1).localY, res.hazardList.at(res.numHazards-1).localX, res.hazardList.at(res.numHazards-1).localY, RAD2DEG*globalMapPathHazardsPolygonHeading);
            res.hazardList.at(res.numHazards-1).height = globalMap.at(layerToString(_objectHeight), *it);
        }
    }
    return true;
}

bool MapManager::searchLocalMapInfoCallback(messages::SearchLocalMapInfo::Request &req, messages::SearchLocalMapInfo::Response &res) // need to test
{
    if(searchLocalMapExists)
    {
        res.value = searchLocalMap.atPosition(layerToString(static_cast<MAP_LAYERS_T>(req.layer)), grid_map::Position(req.x, req.y));
        return true;
    }
    else return false;
}

void MapManager::keyframesCallback(const messages::KeyframeList::ConstPtr &msg) // tested
{
    keyframes = *msg;
    writeSatMapIntoGlobalMap();
    writeKeyframesIntoGlobalMap();
}

void MapManager::globalPoseCallback(const messages::RobotPose::ConstPtr &msg) // need to implement hsm to publish this
{
    globalPose = *msg;
    if(globalPose.northAngle != previousNorthAngle)
    {
        updateNorthTransformedMapData();
        previousNorthAngle = globalPose.northAngle;
    }
    currentROIMsg.currentROINum = globalMap.atPosition(layerToString(_roiNum),grid_map::Position(globalPose.x, globalPose.y));
    currentROIPub.publish(currentROIMsg);
}

void MapManager::keyframeRelPoseCallback(const messages::SLAMPoseOut::ConstPtr &msg) // need to implement output in SLAM for this
{
    keyframeRelPose = *msg;
}

void MapManager::cvSamplesFoundCallback(const messages::CVSamplesFound::ConstPtr &msg) // need to figure out how the sample prob update works
{
    cvSamplesFoundMsg = *msg;
    if(searchLocalMapExists/* && (keyframeRelPose.keyframeIndex == currentROIMsg.currentROINum)*/) // Do we want this condition?
    {
        //donutSmash(MAP_SAMPLE_PROB_LAYERS_START_INDEX, MAP_SAMPLE_PROB_LAYERS_END_INDEX, searchLocalMap, grid_map::Position(keyframeRelPose.keyframeRelX, keyframeRelPose.keyframeRelY));
        //addFoundSamples(MAP_SAMPLE_PROB_LAYERS_START_INDEX, MAP_SAMPLE_PROB_LAYERS_END_INDEX, searchLocalMap, grid_map::Position(keyframeRelPose.keyframeRelX, keyframeRelPose.keyframeRelY), keyframeRelPose.keyframeRelHeading);
    }
}

void MapManager::gridMapResetLayers(int startIndex, int endIndex, grid_map::GridMap &map) // tested
{
    for(int i=startIndex; i<=endIndex; i++)
    {
        if(static_cast<MAP_LAYERS_T>(i)==_driveability) map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), (float)_impassable);
        else map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), 0.0);
    }
}

void MapManager::gridMapAddLayers(int layerStartIndex, int layerEndIndex, grid_map::GridMap &map) // tested
{
    for(int i=layerStartIndex; i<=layerEndIndex; i++)
    {
        if(static_cast<MAP_LAYERS_T>(i)==_driveability) map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), (float)_impassable);
        else map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), 0.0);
    }
}

void MapManager::rotateCoord(float origX, float origY, float &newX, float &newY, float angleDeg)
{
    newX = origX*cos(DEG2RAD*angleDeg)+origY*sin(DEG2RAD*angleDeg);
    newY = -origX*sin(DEG2RAD*angleDeg)+origY*cos(DEG2RAD*angleDeg);
}

void MapManager::rotateCoord(double origX, double origY, double &newX, double &newY, double angleDeg)
{
    newX = origX*cos(DEG2RAD*angleDeg)+origY*sin(DEG2RAD*angleDeg);
    newY = -origX*sin(DEG2RAD*angleDeg)+origY*cos(DEG2RAD*angleDeg);
}

void MapManager::writeSatMapIntoGlobalMap() // tested
{
    gridMapResetLayers((int)_slope, (int)_driveability, globalMap);
    // Slope
    for(int i=0; i<slopeNumRows; i++)
    {
        for(int j=0; j<slopeNumCols; j++)
        {
            rotateCoord((float)(j*slopeMapRes+slopeMapRes/2.0) - satMapStartE, (float)(i*slopeMapRes+slopeMapRes/2.0) - satMapStartS, satMapToGlobalMapPos[0], satMapToGlobalMapPos[1], globalPose.northAngle-90.0);
            //satMapToGlobalMapPos[0] = (float)(j*slopeMapRes+slopeMapRes/2.0) - satMapStartE;
            //satMapToGlobalMapPos[1] = (float)(i*slopeMapRes+slopeMapRes/2.0) - satMapStartS;
            if(globalMap.isInside(satMapToGlobalMapPos)) globalMap.atPosition(layerToString(_slope), satMapToGlobalMapPos) = slopeMap[i][j]; // *** Got to figure out if this is correct
        }
    }
    // Driveability
    for(int i=0; i<slopeNumRows; i++)
    {
        for(int j=0; j<slopeNumCols; j++)
        {
            rotateCoord((float)(j*driveabilityMapRes+driveabilityMapRes/2.0) - satMapStartE, (float)(i*driveabilityMapRes+driveabilityMapRes/2.0) - satMapStartS, satMapToGlobalMapPos[0], satMapToGlobalMapPos[1], globalPose.northAngle-90.0);
            //satMapToGlobalMapPos[0] = (float)(j*driveabilityMapRes+driveabilityMapRes/2.0) - satMapStartE;
            //satMapToGlobalMapPos[1] = (float)(i*driveabilityMapRes+driveabilityMapRes/2.0) - satMapStartS;
            if(globalMap.isInside(satMapToGlobalMapPos))
            {
                if(driveabilityMap[i][j]==1) globalMap.atPosition(layerToString(_driveability), satMapToGlobalMapPos) = (float)_impassable;
                else globalMap.atPosition(layerToString(_driveability), satMapToGlobalMapPos) = (float)_noObject;
            }
        }
    }
    /*grid_map::GridMapRosConverter::toMessage(globalMap, globalMapMsg);
    globalMapPub.publish(globalMapMsg);
    ros::Duration(3).sleep();*/
    smoothDriveabilityLayer();
    grid_map::GridMapRosConverter::toMessage(globalMap, globalMapMsg);
    globalMapPub.publish(globalMapMsg);
}

void MapManager::writeKeyframesIntoGlobalMap()
{
    ++keyframeWriteIntoGlobalMapSerialNum;
    gridMapResetLayers(MAP_KEYFRAME_LAYERS_START_INDEX+1, MAP_KEYFRAME_LAYERS_END_INDEX, globalMap);
    for(int i=0; i<keyframes.keyframeList.size(); i++)
    {
        grid_map::GridMapRosConverter::fromMessage(keyframes.keyframeList.at(i).map,currentKeyframe);
        keyframeHeading = keyframes.keyframeList.at(i).heading;
        keyframeXPos = keyframes.keyframeList.at(i).x;
        keyframeYPos = keyframes.keyframeList.at(i).y;
        //ROS_INFO("currentKeyframe.at = %f",currentKeyframe.atPosition(layerToString(_driveability), grid_map::Position(30.0,30.0)));
        for(grid_map::GridMapIterator it(currentKeyframe); !it.isPastEnd(); ++it)
        {
            currentKeyframe.getPosition(*it, keyframeCoord);
            rotateCoord(keyframeCoord[0], keyframeCoord[1], globalTransformCoord[0], globalTransformCoord[1], keyframeHeading);
            globalTransformCoord[0] += keyframeXPos;
            globalTransformCoord[1] += keyframeYPos;
            //globalTransformCoord[0] = keyframeCoord[0]*cos(DEG2RAD*keyframeHeading)+keyframeCoord[1]*sin(DEG2RAD*keyframeHeading)+keyframeXPos;
            //globalTransformCoord[1] = -keyframeCoord[0]*sin(DEG2RAD*keyframeHeading)+keyframeCoord[1]*cos(DEG2RAD*keyframeHeading)+keyframeYPos;
            if(globalMap.isInside(globalTransformCoord))
            {
                for(int j=MAP_KEYFRAME_LAYERS_START_INDEX; j<=MAP_KEYFRAME_LAYERS_END_INDEX; j++)
                {
                    currentCellValue = globalMap.atPosition(layerToString(static_cast<MAP_LAYERS_T>(j)),globalTransformCoord);
                    possibleNewCellValue = currentKeyframe.at(layerToString(static_cast<MAP_LAYERS_T>(j)),*it);
                    //if(j==(int)_driveability && possibleNewCellValue!=0.0) ROS_INFO("possibleNewCellValue = %f, currentCellValue = %f, currentSerialNum = %f, newSerialNum = %f",possibleNewCellValue, currentCellValue, globalMap.atPosition(layerToString(_keyframeWriteIntoGlobalMapSerialNum), globalTransformCoord),(float)keyframeWriteIntoGlobalMapSerialNum);
                    if((static_cast<MAP_LAYERS_T>(j)==_driveability) || (static_cast<MAP_LAYERS_T>(j)==_reflectivity))
                    {
                        if((possibleNewCellValue > currentCellValue) || (globalMap.atPosition(layerToString(_keyframeWriteIntoGlobalMapSerialNum), globalTransformCoord) != (float)keyframeWriteIntoGlobalMapSerialNum))
                            globalMap.atPosition(layerToString(static_cast<MAP_LAYERS_T>(j)),globalTransformCoord) = possibleNewCellValue;
                    }
                    else if(static_cast<MAP_LAYERS_T>(j)==_objectHeight)
                    {
                        if(currentKeyframe.at(layerToString(_driveability),*it) == _noObject) globalMap.atPosition(layerToString(static_cast<MAP_LAYERS_T>(j)),globalTransformCoord) = 0;
                        else
                        {
                            if((possibleNewCellValue < currentCellValue) || (globalMap.atPosition(layerToString(_keyframeWriteIntoGlobalMapSerialNum), globalTransformCoord) != (float)keyframeWriteIntoGlobalMapSerialNum))
                                globalMap.atPosition(layerToString(static_cast<MAP_LAYERS_T>(j)),globalTransformCoord) = possibleNewCellValue;
                        }
                    }
                }
                globalMap.atPosition(layerToString(_keyframeWriteIntoGlobalMapSerialNum), globalTransformCoord) = (float)keyframeWriteIntoGlobalMapSerialNum;
            }
        }
    }
    //ROS_INFO("globalMap.at = %f",globalMap.atPosition(layerToString(_driveability), grid_map::Position(50.0,45.0)));
    grid_map::GridMapRosConverter::toMessage(globalMap, globalMapMsg);
    globalMapPub.publish(globalMapMsg);
}

void MapManager::northTransformROIs() // needs tested
{
    for(int i=0; i<regionsOfInterest.size(); i++)
    {
        rotateCoord(regionsOfInterest.at(i).e, regionsOfInterest.at(i).s, regionsOfInterest.at(i).x, regionsOfInterest.at(i).y, globalPose.northAngle-90.0);
    }
}

void MapManager::updateNorthTransformedMapData() // tested* ^^^
{
    writeSatMapIntoGlobalMap();
    writeKeyframesIntoGlobalMap();
    northTransformROIs();
}

void MapManager::smoothDriveabilityLayer() // tested
{
    float currentValue;
    int sumNeighborsDifferentFromCurrentValue;
    for(int i=0; i<globalMapRowsCols[0]; i++)
    {
        for(int j=0; j<globalMapRowsCols[1]; j++)
        {
            //ROS_INFO("i = %i; j = %i",i,j);
            //ROS_INFO("rows = %i; cols = %i",globalMapRowsCols[0], globalMapRowsCols[1]);
            sumNeighborsDifferentFromCurrentValue = 0;
            currentValue = globalMap.get(layerToString(_driveability))(i,j);
            if(((i-1)<0) && ((j-1)<0)) // top left corner
            {
                //ROS_INFO("top left corner");
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // tl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // tc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j+1)!=currentValue; // tr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // ml
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j+1)!=currentValue; // mr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j)!=currentValue; // bl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j)!=currentValue; // bc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j+1)!=currentValue; // br
                if(sumNeighborsDifferentFromCurrentValue >= smoothDriveabilityNumNeighborsToChangeValue)
                {
                    if(currentValue==(float)_impassable) globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_noObject;
                    else globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_impassable;
                }
                else globalMapTemp.get(layerToString(_driveability))(i,j) = currentValue;
            }
            else if(((i+1)>=globalMapRowsCols[0]) && ((j-1)<0)) // bottom left corner
            {
                //ROS_INFO("bottom left corner");
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j)!=currentValue; // tl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j)!=currentValue; // tc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j+1)!=currentValue; // tr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // ml
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j+1)!=currentValue; // mr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // bl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // bc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j+1)!=currentValue; // br
                if(sumNeighborsDifferentFromCurrentValue >= smoothDriveabilityNumNeighborsToChangeValue)
                {
                    if(currentValue==(float)_impassable) globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_noObject;
                    else globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_impassable;
                }
                else globalMapTemp.get(layerToString(_driveability))(i,j) = currentValue;
            }
            else if(((i-1)<0) && ((j+1)>=globalMapRowsCols[1])) // top right corner
            {
                //ROS_INFO("top right corner");
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j-1)!=currentValue; // tl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // tc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // tr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j-1)!=currentValue; // ml
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // mr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j-1)!=currentValue; // bl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j)!=currentValue; // bc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j)!=currentValue; // br
                if(sumNeighborsDifferentFromCurrentValue >= smoothDriveabilityNumNeighborsToChangeValue)
                {
                    if(currentValue==(float)_impassable) globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_noObject;
                    else globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_impassable;
                }
                else globalMapTemp.get(layerToString(_driveability))(i,j) = currentValue;
            }
            else if(((i+1)>=globalMapRowsCols[0]) && ((j+1)>=globalMapRowsCols[1])) // bottom right corner
            {
                //ROS_INFO("bottom right corner");
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j-1)!=currentValue; // tl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j)!=currentValue; // tc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j)!=currentValue; // tr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j-1)!=currentValue; // ml
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // mr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j-1)!=currentValue; // bl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // bc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // br
                if(sumNeighborsDifferentFromCurrentValue >= smoothDriveabilityNumNeighborsToChangeValue)
                {
                    if(currentValue==(float)_impassable) globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_noObject;
                    else globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_impassable;
                }
                else globalMapTemp.get(layerToString(_driveability))(i,j) = currentValue;
            }
            else if(((i-1)<0) && ((j-1)>=0) && ((j+1)<globalMapRowsCols[1])) // top edge
            {
                //ROS_INFO("top edge");
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j-1)!=currentValue; // tl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // tc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j+1)!=currentValue; // tr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j-1)!=currentValue; // ml
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j+1)!=currentValue; // mr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j-1)!=currentValue; // bl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j)!=currentValue; // bc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j+1)!=currentValue; // br
                if(sumNeighborsDifferentFromCurrentValue >= smoothDriveabilityNumNeighborsToChangeValue)
                {
                    if(currentValue==(float)_impassable) globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_noObject;
                    else globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_impassable;
                }
                else globalMapTemp.get(layerToString(_driveability))(i,j) = currentValue;
            }
            else if(((i+1)>=globalMapRowsCols[0]) && ((j-1)>=0) && ((j+1)<globalMapRowsCols[1])) // bottom edge
            {
                //ROS_INFO("bottom edge");
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j-1)!=currentValue; // tl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j)!=currentValue; // tc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j+1)!=currentValue; // tr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j-1)!=currentValue; // ml
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j+1)!=currentValue; // mr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j-1)!=currentValue; // bl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // bc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j+1)!=currentValue; // br
                if(sumNeighborsDifferentFromCurrentValue >= smoothDriveabilityNumNeighborsToChangeValue)
                {
                    if(currentValue==(float)_impassable) globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_noObject;
                    else globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_impassable;
                }
                else globalMapTemp.get(layerToString(_driveability))(i,j) = currentValue;
            }
            else if(((i-1)>=0) && ((i+1)<globalMapRowsCols[0]) && ((j-1)<0)) // left edge
            {
                //ROS_INFO("left edge");
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j)!=currentValue; // tl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j)!=currentValue; // tc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j+1)!=currentValue; // tr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // ml
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j+1)!=currentValue; // mr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j)!=currentValue; // bl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j)!=currentValue; // bc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j+1)!=currentValue; // br
                if(sumNeighborsDifferentFromCurrentValue >= smoothDriveabilityNumNeighborsToChangeValue)
                {
                    if(currentValue==(float)_impassable) globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_noObject;
                    else globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_impassable;
                }
                else globalMapTemp.get(layerToString(_driveability))(i,j) = currentValue;
            }
            else if(((i-1)>=0) && ((i+1)<globalMapRowsCols[0]) && ((j+1)>=globalMapRowsCols[1])) // right edge
            {
                //ROS_INFO("right edge");
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j-1)!=currentValue; // tl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j)!=currentValue; // tc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j)!=currentValue; // tr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j-1)!=currentValue; // ml
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j)!=currentValue; // mr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j-1)!=currentValue; // bl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j)!=currentValue; // bc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j)!=currentValue; // br
                if(sumNeighborsDifferentFromCurrentValue >= smoothDriveabilityNumNeighborsToChangeValue)
                {
                    if(currentValue==(float)_impassable) globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_noObject;
                    else globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_impassable;
                }
                else globalMapTemp.get(layerToString(_driveability))(i,j) = currentValue;
            }
            else // internal
            {
                //ROS_INFO("internal");
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j-1)!=currentValue; // tl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j)!=currentValue; // tc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i-1,j+1)!=currentValue; // tr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j-1)!=currentValue; // ml
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i,j+1)!=currentValue; // mr
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j-1)!=currentValue; // bl
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j)!=currentValue; // bc
                sumNeighborsDifferentFromCurrentValue += globalMap.get(layerToString(_driveability))(i+1,j+1)!=currentValue; // br
                if(sumNeighborsDifferentFromCurrentValue >= smoothDriveabilityNumNeighborsToChangeValue)
                {
                    if(currentValue==(float)_impassable) globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_noObject;
                    else globalMapTemp.get(layerToString(_driveability))(i,j) = (float)_impassable;
                }
                else globalMapTemp.get(layerToString(_driveability))(i,j) = currentValue;
            }
        }
    }
    globalMap.addDataFrom(globalMapTemp, false, true, false, std::vector<std::string>(1,layerToString(_driveability)));
}

void MapManager::calculateGlobalMapSize()
{
    float candidateSize;
    float bestCandidateSize = 0.0;
    candidateSize = hypot(satMapStartE, satMapStartS);
    ROS_INFO("candidateSize = %f",candidateSize);
    ROS_INFO("bestCandidateSize = %f",bestCandidateSize);
    if(candidateSize>bestCandidateSize) bestCandidateSize = candidateSize;
    candidateSize = hypot(satMapStartE, satMapSize[1] - satMapStartS);
    ROS_INFO("candidateSize = %f",candidateSize);
    ROS_INFO("bestCandidateSize = %f",bestCandidateSize);
    if(candidateSize>bestCandidateSize) bestCandidateSize = candidateSize;
    candidateSize = hypot(satMapSize[0] - satMapStartE, satMapStartS);
    ROS_INFO("candidateSize = %f",candidateSize);
    ROS_INFO("bestCandidateSize = %f",bestCandidateSize);
    if(candidateSize>bestCandidateSize) bestCandidateSize = candidateSize;
    candidateSize = hypot(satMapSize[0] - satMapStartE, satMapSize[1] - satMapStartS);
    ROS_INFO("candidateSize = %f",candidateSize);
    ROS_INFO("bestCandidateSize = %f",bestCandidateSize);
    if(candidateSize>bestCandidateSize) bestCandidateSize = candidateSize;
    ROS_INFO("candidateSize = %f",candidateSize);
    ROS_INFO("bestCandidateSize = %f",bestCandidateSize);
    globalMapSize[0] = bestCandidateSize*2.0;
    globalMapSize[1] = bestCandidateSize*2.0;
    ROS_INFO("globalMapSize[0] = %f, [1] = %f",globalMapSize[0],globalMapSize[1]);
}
