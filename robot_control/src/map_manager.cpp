#include <robot_control/map_manager.h>

MapManager::MapManager()
    : globalMap({"sampleProb","hazard"}) // FIX THIS !!!!!!!!! somehow incorporate with new technique in common header
{
    roiServ = nh.advertiseService("/control/mapmanager/regionsofinterest", &MapManager::listROI, this);
    modROIServ = nh.advertiseService("/control/mapmanager/modifyroi", &MapManager::modROI, this);
    mapROIServ = nh.advertiseService("/control/mapmanager/roigridmap", &MapManager::mapROI, this);
    createROIKeyframeClient = nh.serviceClient<messages::CreateROIKeyframe>("/slam/keyframesnode/createroikeyframe");
    keyframesSub = nh.subscribe<messages::KeyframeList>("/slam/keyframesnode/keyframelist", 1, &MapManager::keyframesCallback, this);
    globalPoseSub = nh.subscribe<messages::RobotPose>("/hsm/masterexec/globalpose", 1, &MapManager::globalPoseCallback, this);
    keyframeRelPoseSub = nh.subscribe<messages::SLAMPoseOut>("/slam/keyframesnode/slamposeout", 1, &MapManager::keyframeRelPoseCallback, this);
    currentROIPub = nh.advertise<robot_control::CurrentROI>("/control/mapmanager/currentroi", 1);
    currentROIMsg.currentROINum = 0; // 0 means in no ROI
    searchLocalMapROINum = 0;

    // Temporary ROIs. Rectangle around starting platform
    ROI.x = 8.0;
    ROI.y = 5.0;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.x = -8.0;
    ROI.y = 5.0;
    ROI.purpleProb = 500;
    ROI.redProb = 500;
    ROI.blueProb = 500;
    ROI.silverProb = 500;
    ROI.brassProb = 500;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.x = -8.0;
    ROI.y = -5.0;
    ROI.purpleProb = 400;
    ROI.redProb = 400;
    ROI.blueProb = 400;
    ROI.silverProb = 400;
    ROI.brassProb = 400;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.x = 8.0;
    ROI.y = -5.0;
    ROI.purpleProb = 300;
    ROI.redProb = 300;
    ROI.blueProb = 300;
    ROI.silverProb = 300;
    ROI.brassProb = 300;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);

    // Temporary ROIs. Search in front of library
    /*ROI.x = 35.0826;
    ROI.y = 20.9706;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.x = 34.9990;
    ROI.y = 28.0289;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.x = 42.0208;
    ROI.y = 28.1840;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.x = 48.9591;
    ROI.y = 28.0289;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.x = 48.9591;
    ROI.y = 21.0482;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.x = 42.1044;
    ROI.y = 21.2033;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.x = 41.9373;
    ROI.y = 34.9320;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.x = 45.7825;
    ROI.y = 31.5968;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.x = 45.4482;
    ROI.y = 24.3059;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.x = 38.5099;
    ROI.y = 24.5385;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);
    ROI.x = 38.5935;
    ROI.y = 31.5968;
    ROI.purpleProb = 600;
    ROI.redProb = 600;
    ROI.blueProb = 600;
    ROI.silverProb = 600;
    ROI.brassProb = 600;
    ROI.searched = false;
    regionsOfInterest.push_back(ROI);*/

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

    globalMap.setFrameId("map");
    globalMap.setGeometry(grid_map::Length(300.0, 200.0), mapResolution, grid_map::Position(12.0, 65.0));
    gridMapAddLayers(0, MAP_KEYFRAME_LAYERS_END_INDEX, globalMap);

    searchLocalMap.setFrameId("map");
    searchLocalMap.setGeometry(grid_map::Length(searchLocalMapLength, searchLocalMapWidth), mapResolution, grid_map::Position(0.0, 0.0));
    gridMapAddLayers(0, NUM_MAP_LAYERS-1, searchLocalMap);

    currentKeyframe.setFrameId("map");
    ROIKeyframe.setFrameId("map");
}

bool MapManager::listROI(robot_control::RegionsOfInterest::Request &req, robot_control::RegionsOfInterest::Response &res)
{
    res.ROIList = regionsOfInterest;
    ROS_INFO("sent ROI");
    return true;
}

bool MapManager::modROI(robot_control::ModifyROI::Request &req, robot_control::ModifyROI::Response &res)
{
    if(req.setSearchedROI) regionsOfInterest.at(req.numSearchedROI).searched = req.searchedROIState;
    if(req.addNewROI)
    {
        regionsOfInterest.push_back(req.newROI);
    }
    ROS_INFO("modified ROI");
    return true;
}

bool MapManager::mapROI(messages::ROIGridMap::Request &req, messages::ROIGridMap::Response &res)
{
    //res.map = // retreive map segment based on ROI index
    return true;
}

bool MapManager::searchMapCallback(robot_control::SearchMap::Request &req, robot_control::SearchMap::Response &res)
{
    if(req.createMap && !req.deleteMap)
    {
        createROIKeyframeSrv.request.roiIndex = req.roiIndex;
        if(createROIKeyframeClient.call(createROIKeyframeSrv)) ROS_DEBUG("MAP MANAGER: createROIKeyframe service call successful"); // call createROIKeyframe service
        else {ROS_ERROR("MAP MANAGER createROIKeyframe service call unsuccessful"); return false;}
        grid_map::GridMapRosConverter::fromMessage(createROIKeyframeSrv.response.keyframe.map, ROIKeyframe);
        searchLocalMapToROIAngle = RAD2DEG*atan2(regionsOfInterest.at(req.roiIndex).y, regionsOfInterest.at(req.roiIndex).x) - fmod(createROIKeyframeSrv.response.keyframe.heading, 360.0);
        sigmaROIX = regionsOfInterest.at(req.roiIndex).radialAxis/2.0/numSigmasROIAxis;
        sigmaROIY = regionsOfInterest.at(req.roiIndex).tangentialAxis/2.0/numSigmasROIAxis;
        for(grid_map::GridMapIterator it(searchLocalMap); !it.isPastEnd(); ++it)
        {
            searchLocalMap.getPosition(*it, searchLocalMapCoord);
            ROIX = searchLocalMapCoord[0]*cos(DEG2RAD*searchLocalMapToROIAngle)-searchLocalMapCoord[1]*sin(DEG2RAD*searchLocalMapToROIAngle);
            ROIY = searchLocalMapCoord[0]*sin(DEG2RAD*searchLocalMapToROIAngle)+searchLocalMapCoord[1]*cos(DEG2RAD*searchLocalMapToROIAngle);
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
    }
    else if(req.deleteMap && !req.createMap)
    {
        searchLocalMap.clearBasic();
    }
    return true;
}

void MapManager::keyframesCallback(const messages::KeyframeList::ConstPtr &msg)
{
    keyframes = *msg;
    clearGlobalMapLayers(MAP_KEYFRAME_LAYERS_START_INDEX, MAP_KEYFRAME_LAYERS_END_INDEX);
    for(int i=0; i<keyframes.keyframeList.size(); i++)
    {
        grid_map::GridMapRosConverter::fromMessage(keyframes.keyframeList.at(i).map,currentKeyframe);
        //for(int j=0; j<NUM_MAP_LAYERS; j++) keyframeTransform.add(layerToString(static_cast<MAP_LAYERS_T>(j)),0);
        //keyframeOriginalXLen = currentKeyframe.getLength().x();
        //keyframeOriginalYLen = currentKeyframe.getLength().y();
        keyframeHeading = keyframes.keyframeList.at(i).heading;
        keyframeXPos = keyframes.keyframeList.at(i).x;
        keyframeYPos = keyframes.keyframeList.at(i).y;
        //keyframeTransformXLen = keyframeOriginalXLen*cos(DEG2RAD*fmod(keyframeTransformHeading,90.0))+fabs(keyframeOriginalYLen*sin(DEG2RAD*fmod(keyframeTransformHeading,90.0)));
        //keyframeTransformYLen = keyframeOriginalYLen*cos(DEG2RAD*fmod(keyframeTransformHeading,90.0))+fabs(keyframeOriginalXLen*sin(DEG2RAD*fmod(keyframeTransformHeading,90.0)));
        //keyframeTransform.setGeometry(grid_map::Length(keyframeTransformXLen,keyframeTransformYLen), mapResolution, grid_map::Position(0.0,0.0));
        for(grid_map::GridMapIterator it(currentKeyframe); !it.isPastEnd(); ++it)
        {
            currentKeyframe.getPosition(*it, keyframeCoord);
            globalTransformCoord[0] = keyframeCoord[0]*cos(DEG2RAD*keyframeHeading)-keyframeCoord[1]*sin(DEG2RAD*keyframeHeading)+keyframeXPos;
            globalTransformCoord[1] = keyframeCoord[0]*sin(DEG2RAD*keyframeHeading)+keyframeCoord[1]*cos(DEG2RAD*keyframeHeading)+keyframeYPos;
            for(int j=MAP_KEYFRAME_LAYERS_START_INDEX; j<=MAP_KEYFRAME_LAYERS_END_INDEX; j++)
            {
                globalMap.atPosition(layerToString(static_cast<MAP_LAYERS_T>(j)),globalTransformCoord) = currentKeyframe.at(layerToString(static_cast<MAP_LAYERS_T>(j)),*it);
            }
        }
    }
}

void MapManager::globalPoseCallback(const messages::RobotPose::ConstPtr &msg)
{
    globalPose = *msg;
    currentROIMsg.currentROINum = globalMap.atPosition(layerToString(_roiNum),grid_map::Position(globalPose.x, globalPose.y));
    currentROIPub.publish(currentROIMsg);
}

void MapManager::keyframeRelPoseCallback(const messages::SLAMPoseOut::ConstPtr &msg)
{
    keyframeRelPose = *msg;
}

void MapManager::clearGlobalMapLayers(int startIndex, int endIndex)
{
    for(int i=startIndex; i<=endIndex; i++) globalMap.clear(layerToString(static_cast<MAP_LAYERS_T>(i)));
}

void MapManager::gridMapAddLayers(int layerStartIndex, int layerEndIndex, grid_map::GridMap &map)
{
    for(int i=layerStartIndex; i<=layerEndIndex; i++)
    {
        map.add(layerToString(static_cast<MAP_LAYERS_T>(i)));
    }
}
