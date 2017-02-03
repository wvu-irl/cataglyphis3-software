#include <robot_control/map_manager.h>

MapManager::MapManager()
{
    roiServ = nh.advertiseService("/control/mapmanager/regionsofinterest", &MapManager::listROI, this);
    modROIServ = nh.advertiseService("/control/mapmanager/modifyroi", &MapManager::modROI, this);
    searchMapServ = nh.advertiseService("/control/mapmanager/searchlocalmap", &MapManager::searchMapCallback, this);
    globalMapPathHazardsServ = nh.advertiseService("/control/mapmanager/globalmappathhazards", &MapManager::globalMapPathHazardsCallback, this);
    searchLocalMapPathHazardsServ = nh.advertiseService("/control/mapmanager/searchlocalmappathhazards", &MapManager::searchLocalMapPathHazardsCallback, this);
    searchLocalMapInfoServ = nh.advertiseService("/control/mapmanager/searchlocalmapinfo", &MapManager::searchLocalMapInfoCallback, this);
    randomSearchWaypointsServ = nh.advertiseService("/control/mapmanager/randomsearchwaypoints", &MapManager::randomSearchWaypointsCallback, this);
    globalMapFullServ = nh.advertiseService("/control/mapmanager/globalmapfull", &MapManager::globalMapFullCallback, this);
    setStartingPlatformServ = nh.advertiseService("/control/mapmanager/setstartingplatform", &MapManager::setStartingPlatformCallback, this);
    createROIHazardMapClient = nh.serviceClient<messages::CreateROIHazardMap>("/lidar/collisiondetection/createroihazardmap");
    keyframesSub = nh.subscribe<messages::KeyframeList>("/slam/keyframesnode/keyframelist", 1, &MapManager::keyframesCallback, this);
    globalPoseSub = nh.subscribe<messages::RobotPose>("/hsm/masterexec/globalpose", 1, &MapManager::globalPoseCallback, this);
    keyframeRelPoseSub = nh.subscribe<messages::SLAMPoseOut>("/slam/localizationnode/slamposeout", 1, &MapManager::keyframeRelPoseCallback, this);
    cvSamplesFoundSub = nh.subscribe<messages::CVSamplesFound>("/vision/samplesearch/samplesearchout", 1, &MapManager::cvSamplesFoundCallback, this);
    currentROIPub = nh.advertise<robot_control::CurrentROI>("/control/mapmanager/currentroi", 1);
    globalMapPub = nh.advertise<grid_map_msgs::GridMap>("/control/mapmanager/globalmapviz", 1);
    searchLocalMapPub = nh.advertise<grid_map_msgs::GridMap>("/control/mapmanager/searchlocalmapviz", 1);
    roisModifiedPub = nh.advertise<robot_control::ROIList>("/control/mapmanager/roimodifiedlist", 1);
    currentROIMsg.currentROINum = 0; // 0 means in no ROI
    searchLocalMapROINum = 0;
    keyframeWriteIntoGlobalMapSerialNum = 0;
    searchLocalMapExists = false;
    mapPathHazardsVertices.resize(4);
    globalPose.northAngle = 90.0; // degrees. initial guess
    previousNorthAngle = 89.999; // degrees. different than actual north angle to force update first time through
    // By default, assume starting platform location 2
    startingPlatformLocation = 2;
    satMapStartE1Offset = 0.0;
    satMapStartS1Offset = 0.0;
    satMapStartE2Offset = 0.0;
    satMapStartS2Offset = 0.0;
    satMapStartE3Offset = 0.0;
    satMapStartS3Offset = 0.0;
    setStartingPlatform();
    srand(time(NULL));

// Square around starting platform. Must initialize with north angle = 90.0 degrees
//#include <robot_control/square_rois.h>

#ifdef EVANSDALE
// Dense ROIs to search directly in front of library
//#include <robot_control/evansdale_short_dense_rois.h>

// Limited set of ROIs covering eastern half of Evansdale in front of library
//#include <robot_control/evansdale_library_rois.h>

// Full set of ROIs covering Evansdale in front of library and engineering
#include <robot_control/evansdale_full_rois.h>
#endif // EVANSDALE

#ifdef WPI
// WPI Institute Park ROIs
#include <robot_control/wpi_rois.h>
#endif // WPI

#ifdef TEST_FIELD
// Test field at WPI ROIs
#include <robot_control/test_field_rois.h>
#endif // TEST_FIELD

#ifdef CHESTNUT_RIDGE
// ROIs for Chestnut Ridge Park
#include <robot_control/chestnut_ridge_rois.h>
#endif // CHESTNUT_RIDGE

    computeROIEastSouth();

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
/*#ifdef EVANSDALE
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
#endif // INSTUTUTE_PARK*/
    satMapSize[0] = ((float)driveabilityNumCols)*driveabilityMapRes;
    satMapSize[1] = ((float)driveabilityNumRows)*driveabilityMapRes;
    globalMapOrigin[0] = 0.0;
    globalMapOrigin[1] = 0.0;
    setStartingPlatform();
    calculateGlobalMapSize();
    initializeGlobalMap();
    writeSatMapIntoGlobalMap();
    globalMapPathHazardsPolygon.setFrameId(globalMap.getFrameId());

    searchLocalMap.setFrameId("map");
    searchLocalMap.setGeometry(grid_map::Length(searchLocalMapLength, searchLocalMapWidth), mapResolution, grid_map::Position(0.0, 0.0));
    gridMapAddLayers(0, NUM_MAP_LAYERS-1, searchLocalMap);
    searchLocalMapPathHazardsPolygon.setFrameId(searchLocalMap.getFrameId());

    currentKeyframe.setFrameId("map");
    ROIKeyframe.setFrameId("map");

    globalSubMap.setFrameId("map");

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
    if(req.setHardLockoutROI) regionsOfInterest.at(req.modROIIndex).hardLockout = req.hardLockoutROIState;
    if(req.setPosES && !req.setPosXY)
    {
        regionsOfInterest.at(req.modROIIndex).e = req.e;
        regionsOfInterest.at(req.modROIIndex).s = req.s;
        rotateCoord(regionsOfInterest.back().e, regionsOfInterest.back().s, regionsOfInterest.back().x, regionsOfInterest.back().y, globalPose.northAngle-90.0);
    }
    if(req.setPosXY && !req.setPosES)
    {
        regionsOfInterest.at(req.modROIIndex).x = req.x;
        regionsOfInterest.at(req.modROIIndex).y = req.y;
        rotateCoord(regionsOfInterest.back().x, regionsOfInterest.back().y, regionsOfInterest.back().e, regionsOfInterest.back().s, -(globalPose.northAngle-90.0));
    }
    if(req.setROISize)
    {
        regionsOfInterest.at(req.modROIIndex).radialAxis = req.radialAxis;
        regionsOfInterest.at(req.modROIIndex).tangentialAxis = req.tangentialAxis;
    }
    if(req.setSampleProps)
    {
        regionsOfInterest.at(req.modROIIndex).sampleProb = req.sampleProb;
        regionsOfInterest.at(req.modROIIndex).sampleSig = req.sampleSig;
        regionsOfInterest.at(req.modROIIndex).whiteProb = req.whiteProb;
        regionsOfInterest.at(req.modROIIndex).silverProb = req.silverProb;
        regionsOfInterest.at(req.modROIIndex).blueOrPurpleProb = req.blueOrPurpleProb;
        regionsOfInterest.at(req.modROIIndex).pinkProb = req.pinkProb;
        regionsOfInterest.at(req.modROIIndex).redProb = req.redProb;
        regionsOfInterest.at(req.modROIIndex).orangeProb = req.orangeProb;
        regionsOfInterest.at(req.modROIIndex).yellowProb = req.yellowProb;
        if(req.editGroup && regionsOfInterest.at(req.modROIIndex).roiGroup != 0)
        {
            for(int i=0; i<regionsOfInterest.size(); i++)
            {
                if(regionsOfInterest.at(i).roiGroup == regionsOfInterest.at(req.modROIIndex).roiGroup) regionsOfInterest.at(i).sampleProb = req.sampleProb;
            }
        }
    }
    if(req.addNewROI)
    {
        regionsOfInterest.push_back(req.newROI);
        if(req.newROIInESNotXY)
            rotateCoord(regionsOfInterest.back().e, regionsOfInterest.back().s, regionsOfInterest.back().x, regionsOfInterest.back().y, globalPose.northAngle-90.0);
        else
            rotateCoord(regionsOfInterest.back().x, regionsOfInterest.back().y, regionsOfInterest.back().e, regionsOfInterest.back().s, -(globalPose.northAngle-90.0));
    }
    if(req.deleteROI)
    {
        regionsOfInterest.erase(regionsOfInterest.begin()+req.numDeleteROI);
    }
    roisModifiedListMsg.ROIList = regionsOfInterest;
    roisModifiedPub.publish(roisModifiedListMsg);
    ROS_INFO("modified ROI");
    return true;
}

bool MapManager::searchMapCallback(robot_control::SearchMap::Request &req, robot_control::SearchMap::Response &res) // tested
{
    float overallROIProb = 0.0;
    if(req.createMap && !req.deleteMap)
    {
        if(searchLocalMapExists) ROS_WARN("MAP MANAGER: tried to create searchLocalMap when it already exists");
        else
        {
            searchLocalMapROINum = req.roiIndex;
            if(createROIHazardMapClient.call(createROIHazardMapSrv)) ROS_DEBUG("MAP MANAGER: createROIHazardMap service call successful"); // call createROIHazardMap service
            else {ROS_ERROR("MAP MANAGER: createROIHazardMap service call unsuccessful"); return false;}
            searchLocalMapXPos = globalPose.x;
            searchLocalMapYPos = globalPose.y;
            searchLocalMapHeading = globalPose.heading;
            searchLocalMapExists = true;
            //grid_map::GridMapRosConverter::fromMessage(createROIKeyframeSrv.response.keyframe.map, ROIKeyframe);
            searchLocalMapToROIAngle = RAD2DEG*atan2(regionsOfInterest.at(req.roiIndex).s, regionsOfInterest.at(req.roiIndex).e) - fmod(globalPose.heading, 360.0);
            sigmaROIX = regionsOfInterest.at(req.roiIndex).radialAxis/2.0/numSigmasROIAxis;
            sigmaROIY = regionsOfInterest.at(req.roiIndex).tangentialAxis/2.0/numSigmasROIAxis;
            searchLocalMap.setGeometry(grid_map::Length(regionsOfInterest.at(searchLocalMapROINum).radialAxis*2.0, regionsOfInterest.at(searchLocalMapROINum).tangentialAxis*2.0), mapResolution, grid_map::Position(0.0, 0.0));
            searchLocalMap.add(layerToString(_localMapDriveability), 0.0);
            searchLocalMap.add(layerToString(_sampleProb), 1.0);
            for(int i=0; i<createROIHazardMapSrv.response.x_mean.size(); i++)
            {
                searchLocalMapCoord[0] = createROIHazardMapSrv.response.x_mean.at(i);
                searchLocalMapCoord[1] = createROIHazardMapSrv.response.y_mean.at(i);
                //ROS_INFO("searchLocalMapCoord: x = %f; y = %f",searchLocalMapCoord[0], searchLocalMapCoord[1]);
                //ROIX = searchLocalMapCoord[0]*cos(DEG2RAD*searchLocalMapToROIAngle)+searchLocalMapCoord[1]*sin(DEG2RAD*searchLocalMapToROIAngle);
                //ROIY = -searchLocalMapCoord[0]*sin(DEG2RAD*searchLocalMapToROIAngle)+searchLocalMapCoord[1]*cos(DEG2RAD*searchLocalMapToROIAngle);
                //ROS_INFO("ROIX = %f; ROIY = %f",ROIX, ROIY);
                if(searchLocalMap.isInside(searchLocalMapCoord)) searchLocalMap.atPosition(layerToString(_localMapDriveability), searchLocalMapCoord) = 10.0;
            }
            sampleProbPeak = regionsOfInterest.at(searchLocalMapROINum).sampleProb*percentSampleProbAreaInROI/(2.0*PI*sigmaROIX*sigmaROIY);
            //ROS_INFO("sampleProbPeak = %f",sampleProbPeak);
#ifdef USE_DONUT_SMASH
            for(grid_map::GridMapIterator it(searchLocalMap); !it.isPastEnd(); ++it)
            {
                searchLocalMap.getPosition(*it, searchLocalMapCoord);
                rotateCoord(searchLocalMapCoord[0], searchLocalMapCoord[1], ROIX, ROIY, searchLocalMapToROIAngle);
                searchLocalMap.at(layerToString(_sampleProb), *it) = sampleProbPeak*exp(-(pow(ROIX,2.0)/(2.0*pow(sigmaROIX,2.0))+pow(ROIY,2.0)/(2.0*pow(sigmaROIY,2.0))));                
                overallROIProb += searchLocalMap.at(layerToString(_sampleProb), *it);
            }
#endif // USE_DONUT_SMASH
            //ROS_INFO("initial overall ROI %i prob = %f",searchLocalMapROINum,overallROIProb);
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

bool MapManager::globalMapPathHazardsCallback(messages::MapPathHazards::Request &req, messages::MapPathHazards::Response &res) // tested
{
    globalMapPathHazardsPolygon.removeVertices();
    res.hazardValue = 0.0;
    //ROS_INFO("global start = (%f,%f)",req.xStart,req.yStart);
    //ROS_INFO("global end = (%f,%f)",req.xEnd,req.yEnd);
    mapPathHazardsPolygonHeading = atan2(req.yEnd - req.yStart, req.xEnd - req.xStart); // radians
    mapPathHazardsVertices.at(0)[0] = req.xStart + req.width/2.0*sin(mapPathHazardsPolygonHeading);
    mapPathHazardsVertices.at(0)[1] = req.yStart - req.width/2.0*cos(mapPathHazardsPolygonHeading);
    mapPathHazardsVertices.at(1)[0] = req.xStart - req.width/2.0*sin(mapPathHazardsPolygonHeading);
    mapPathHazardsVertices.at(1)[1] = req.yStart + req.width/2.0*cos(mapPathHazardsPolygonHeading);
    mapPathHazardsVertices.at(2)[0] = req.xEnd - req.width/2.0*sin(mapPathHazardsPolygonHeading);
    mapPathHazardsVertices.at(2)[1] = req.yEnd + req.width/2.0*cos(mapPathHazardsPolygonHeading);
    mapPathHazardsVertices.at(3)[0] = req.xEnd + req.width/2.0*sin(mapPathHazardsPolygonHeading);
    mapPathHazardsVertices.at(3)[1] = req.yEnd - req.width/2.0*cos(mapPathHazardsPolygonHeading);
    globalMapPathHazardsPolygon.addVertex(mapPathHazardsVertices.at(0));
    globalMapPathHazardsPolygon.addVertex(mapPathHazardsVertices.at(1));
    globalMapPathHazardsPolygon.addVertex(mapPathHazardsVertices.at(2));
    globalMapPathHazardsPolygon.addVertex(mapPathHazardsVertices.at(3));
    mapPathHazardNumCellsInPolygon = 0;
    for(grid_map::PolygonIterator it(globalMap, globalMapPathHazardsPolygon); !it.isPastEnd(); ++it)
    {
        mapPathHazardValue = globalMap.at(layerToString(_satDriveability), *it);
        if(mapPathHazardValue > 0.0)
        {
            res.hazardValue += mapPathHazardValue;
        }
        mapPathHazardNumCellsInPolygon++;
    }
    if(mapPathHazardNumCellsInPolygon == 0) {res.hazardValue = 0.0; ROS_WARN("globalMapPathHazards numCellsInPolygon == 0");} // Something small as a defualt to avoid divide by zero issue
    else res.hazardValue /= (float)mapPathHazardNumCellsInPolygon;
    return true;
}

bool MapManager::searchLocalMapPathHazardsCallback(messages::MapPathHazards::Request &req, messages::MapPathHazards::Response &res) // tested
{
    float localXStart;
    float localYStart;
    float localXEnd;
    float localYEnd;
    if(searchLocalMapExists)
    {
        searchLocalMapPathHazardsPolygon.removeVertices();
        res.hazardValue = 0.0;
        // Transform global x,y into searchLocalMap x,y
        rotateCoord(req.xStart - searchLocalMapXPos, req.yStart - searchLocalMapYPos, localXStart, localYStart, -searchLocalMapHeading);
        rotateCoord(req.xEnd - searchLocalMapXPos, req.yEnd - searchLocalMapYPos, localXEnd, localYEnd, -searchLocalMapHeading);
        //ROS_INFO("searchLocalMap = (%f,%f)",searchLocalMapXPos, searchLocalMapYPos);
        //ROS_INFO("searchLocalMap heading = %f",searchLocalMapHeading);
        //ROS_INFO("global start = (%f,%f)",req.xStart,req.yStart);
        //ROS_INFO("global end = (%f,%f)",req.xEnd,req.yEnd);
        //ROS_INFO("local start = (%f,%f)",localXStart,localYStart);
        //ROS_INFO("local end = (%f,%f)",localXEnd,localYEnd);
        mapPathHazardsPolygonHeading = atan2(localYEnd - localYStart, localXEnd - localXStart); // radians
        mapPathHazardsVertices.at(0)[0] = localXStart + req.width/2.0*sin(mapPathHazardsPolygonHeading);
        mapPathHazardsVertices.at(0)[1] = localYStart - req.width/2.0*cos(mapPathHazardsPolygonHeading);
        mapPathHazardsVertices.at(1)[0] = localXStart - req.width/2.0*sin(mapPathHazardsPolygonHeading);
        mapPathHazardsVertices.at(1)[1] = localYStart + req.width/2.0*cos(mapPathHazardsPolygonHeading);
        mapPathHazardsVertices.at(2)[0] = localXEnd - req.width/2.0*sin(mapPathHazardsPolygonHeading);
        mapPathHazardsVertices.at(2)[1] = localYEnd + req.width/2.0*cos(mapPathHazardsPolygonHeading);
        mapPathHazardsVertices.at(3)[0] = localXEnd + req.width/2.0*sin(mapPathHazardsPolygonHeading);
        mapPathHazardsVertices.at(3)[1] = localYEnd - req.width/2.0*cos(mapPathHazardsPolygonHeading);
        //ROS_INFO("vertex 0 = (%f,%f)", mapPathHazardsVertices.at(0)[0], mapPathHazardsVertices.at(0)[1]);
        //ROS_INFO("vertex 1 = (%f,%f)", mapPathHazardsVertices.at(1)[0], mapPathHazardsVertices.at(1)[1]);
        //ROS_INFO("vertex 2 = (%f,%f)", mapPathHazardsVertices.at(2)[0], mapPathHazardsVertices.at(2)[1]);
        //ROS_INFO("vertex 3 = (%f,%f)", mapPathHazardsVertices.at(3)[0], mapPathHazardsVertices.at(3)[1]);
        searchLocalMapPathHazardsPolygon.addVertex(mapPathHazardsVertices.at(0));
        searchLocalMapPathHazardsPolygon.addVertex(mapPathHazardsVertices.at(1));
        searchLocalMapPathHazardsPolygon.addVertex(mapPathHazardsVertices.at(2));
        searchLocalMapPathHazardsPolygon.addVertex(mapPathHazardsVertices.at(3));
        mapPathHazardNumCellsInPolygon = 0;
        for(grid_map::PolygonIterator it(searchLocalMap, searchLocalMapPathHazardsPolygon); !it.isPastEnd(); ++it)
        {
            mapPathHazardValue = searchLocalMap.at(layerToString(_localMapDriveability), *it);
            if(mapPathHazardValue > 0.0)
            {
                res.hazardValue += mapPathHazardValue;
            }
            mapPathHazardNumCellsInPolygon++;
        }
        //ROS_INFO("mapPathHazardNumCellsInPolygon = %u",mapPathHazardNumCellsInPolygon);
        if(mapPathHazardNumCellsInPolygon == 0) {res.hazardValue = 0.0; ROS_WARN("searchLocalMapPathHazards numCellsInPolygon == 0");} // Something small as a defualt to avoid divide by zero issue
        else res.hazardValue /= (float)mapPathHazardNumCellsInPolygon;
        return true;
    }
    else return false;
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

bool MapManager::randomSearchWaypointsCallback(robot_control::RandomSearchWaypoints::Request &req, robot_control::RandomSearchWaypoints::Response &res)
{
    if(searchLocalMapExists)
    {
        searchLocalMapNumPoints = searchLocalMap.getSize()[0]*searchLocalMap.getSize()[1];
        possibleRandomWaypointValues.resize(searchLocalMapNumPoints);
        possibleRandomWaypointValuesNormalized.resize(searchLocalMapNumPoints);
        numRandomWaypointsToSelect = req.numSearchWaypoints;
        res.waypointList.resize(numRandomWaypointsToSelect);
        rotateCoord(globalPose.x-searchLocalMapXPos, globalPose.y-searchLocalMapYPos, globalPoseToSearchLocalMapPosition[0], globalPoseToSearchLocalMapPosition[1], searchLocalMapHeading);
#ifdef GREEDY_SEARCH_WAYPOINT_SELECTION
        grid_map::Position cellPosition;
        grid_map::Position convolutionCellPosition;
        grid_map::Index bestSearchLocationIndex;
        grid_map::Position bestSearchLocationPosition;
        float maxConvolutionValue = 0.0;
        float convolutionValue;
        float distanceToConvolutionCell;
        unsigned int convolutionNumCells;
        const float tpMaxValue = 0.99;
        const float tpMaxDistance = 5.0; // m
        searchLocalMap.getIndex(grid_map::Position(0.0,0.0), bestSearchLocationIndex); // default to center of ROI if no other best choice found
        for(grid_map::GridMapIterator it(searchLocalMap); !it.isPastEnd(); ++it)
        {
            searchLocalMap.getPosition(*it, cellPosition);
            convolutionVerticies.clear();
            convolutionVerticies.resize(4);
            convolutionVerticies.at(0)[0] = cellPosition[0] + donutSmashSearchRadius;
            convolutionVerticies.at(0)[1] = cellPosition[1] + donutSmashSearchRadius;
            convolutionVerticies.at(1)[0] = cellPosition[0] - donutSmashSearchRadius;
            convolutionVerticies.at(1)[1] = cellPosition[1] + donutSmashSearchRadius;
            convolutionVerticies.at(2)[0] = cellPosition[0] - donutSmashSearchRadius;
            convolutionVerticies.at(2)[1] = cellPosition[1] - donutSmashSearchRadius;
            convolutionVerticies.at(3)[0] = cellPosition[0] + donutSmashSearchRadius;
            convolutionVerticies.at(3)[1] = cellPosition[1] - donutSmashSearchRadius;
            convolutionPolygon.removeVertices();
            convolutionPolygon.addVertex(convolutionVerticies.at(0));
            convolutionPolygon.addVertex(convolutionVerticies.at(1));
            convolutionPolygon.addVertex(convolutionVerticies.at(2));
            convolutionPolygon.addVertex(convolutionVerticies.at(3));
            convolutionValue = 0.0;
            convolutionNumCells = 0;
            for(grid_map::PolygonIterator convolutionIt(searchLocalMap, convolutionPolygon); !convolutionIt.isPastEnd(); ++convolutionIt)
            {
                if(!((*convolutionIt)[0] == (*it)[0] && (*convolutionIt)[1] == (*it)[1]))
                {
                    convolutionNumCells++;
                    searchLocalMap.getPosition(*convolutionIt,convolutionCellPosition);
                    distanceToConvolutionCell = hypot(convolutionCellPosition[0]-cellPosition[0],convolutionCellPosition[1]-cellPosition[1]);
                    convolutionValue += searchLocalMap.at(layerToString(_sampleProb),*convolutionIt)*(tpMaxValue - pow(distanceToConvolutionCell/tpMaxDistance, 2.0));
                }
            }
            //convolutionValue /= convolutionNumCells;
            if(convolutionValue > maxConvolutionValue)
            {
                maxConvolutionValue = convolutionValue;
                bestSearchLocationIndex[0] = (*it)[0];
                bestSearchLocationIndex[1] = (*it)[1];
            }
        }
        searchLocalMap.getPosition(bestSearchLocationIndex,bestSearchLocationPosition);
        res.waypointList.at(0).x = bestSearchLocationPosition[0];
        res.waypointList.at(0).y = bestSearchLocationPosition[1];
        res.waypointList.at(0).sampleProb = searchLocalMap.at(layerToString(_sampleProb), bestSearchLocationIndex);
        res.waypointList.at(0).searchable = true;
        res.waypointList.at(0).unskippable = false;
        res.waypointList.at(0).maxAvoids = maxNormalWaypointAvoidCount;
        res.waypointList.at(0).whiteProb = regionsOfInterest.at(searchLocalMapROINum).whiteProb;
        res.waypointList.at(0).silverProb = regionsOfInterest.at(searchLocalMapROINum).silverProb;
        res.waypointList.at(0).blueOrPurpleProb = regionsOfInterest.at(searchLocalMapROINum).blueOrPurpleProb;
        res.waypointList.at(0).pinkProb = regionsOfInterest.at(searchLocalMapROINum).pinkProb;
        res.waypointList.at(0).redProb = regionsOfInterest.at(searchLocalMapROINum).redProb;
        res.waypointList.at(0).orangeProb = regionsOfInterest.at(searchLocalMapROINum).orangeProb;
        res.waypointList.at(0).yellowProb = regionsOfInterest.at(searchLocalMapROINum).yellowProb;
        rotateCoord(res.waypointList.at(0).x, res.waypointList.at(0).y, res.waypointList.at(0).x, res.waypointList.at(0).y, -searchLocalMapHeading);
        res.waypointList.at(0).x += searchLocalMapXPos;
        res.waypointList.at(0).y += searchLocalMapYPos;
#else
        possibleRandomWaypointValuesSum = 0.0;
        //ROS_INFO("after initial setup");
        for(grid_map::GridMapIterator it(searchLocalMap); !it.isPastEnd(); ++it)
        {
            possibleRandomWaypointValues.at(it.getLinearIndex()) = searchLocalMap.at(layerToString(_sampleProb),*it);
            possibleRandomWaypointValuesSum += possibleRandomWaypointValues.at(it.getLinearIndex());
        }
        //ROS_INFO("after computing values of cells");
        for(int i=0; i<searchLocalMapNumPoints; i++)
        {
            possibleRandomWaypointValuesNormalized.at(i) = possibleRandomWaypointValues.at(i)/possibleRandomWaypointValuesSum;
        }
        //ROS_INFO("after normalizing values");
        numRandomWaypointsSelected = 0;
        numRandomWaypointSearchDistanceCriteriaFailed = 0;
        while(numRandomWaypointsSelected<numRandomWaypointsToSelect)
        {
            //ROS_INFO("numRandomWaypointsSelected = %i",numRandomWaypointsSelected);
            randomValueFloor = 0.0;
            randomValue = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            //ROS_INFO("random value = %f",randomValue);
            for(int j=0; j<searchLocalMapNumPoints; j++)
            {
                if((randomValue >= randomValueFloor) && (randomValue < (randomValueFloor + possibleRandomWaypointValuesNormalized.at(j))))
                {
                    candidateRandomWaypointIndex = j;
                    numRandomWaypointsSelected++;
                    //ROS_INFO("candidateIndex = %i",candidateRandomWaypointIndex);
                    break;
                }
                else randomValueFloor += possibleRandomWaypointValuesNormalized.at(j);
            }
            randomWaypointIndex = grid_map::getIndexFromLinearIndex(candidateRandomWaypointIndex, searchLocalMap.getSize());
            searchLocalMap.getPosition(randomWaypointIndex, randomWaypointPosition);
            if(numRandomWaypointsSelected>1)
            {
                randomWaypointDistanceCriteriaFailed = false;
                for(int j=0; j<(numRandomWaypointsSelected-1); j++)
                {
                    //ROS_INFO("randomWaypoint: x=%f, y=%f\twaypointList.at(%i).x=%f, y=%f", randomWaypointPosition[0], randomWaypointPosition[1], j, res.waypointList.at(j).x, res.waypointList.at(j).y);
                    //ROS_INFO("distance to %i = %f", j, hypot(randomWaypointPosition[0]-res.waypointList.at(j).x, randomWaypointPosition[1]-res.waypointList.at(j).y));
                    if((hypot(randomWaypointPosition[0]-res.waypointList.at(j).x, randomWaypointPosition[1]-res.waypointList.at(j).y) < randomWaypointMinDistance)
                            || (hypot(randomWaypointPosition[0]-globalPoseToSearchLocalMapPosition[0], randomWaypointPosition[1]-globalPoseToSearchLocalMapPosition[1]) < randomWaypointMinDistance))
                    {
                        //ROS_INFO("randomWaypoint distance criterial failed");
                        numRandomWaypointsSelected--;
                        numRandomWaypointSearchDistanceCriteriaFailed++;
                        randomWaypointDistanceCriteriaFailed = true;
                        break;
                    }
                }
            }
            else
            {
                if(hypot(randomWaypointPosition[0]-globalPoseToSearchLocalMapPosition[0], randomWaypointPosition[1]-globalPoseToSearchLocalMapPosition[1]) < randomWaypointMinDistance)
                    {randomWaypointDistanceCriteriaFailed = true; numRandomWaypointsSelected--;}
                else randomWaypointDistanceCriteriaFailed = false;
            }
            if(!randomWaypointDistanceCriteriaFailed)
            {
                //ROS_INFO("randomWaypoint passed, xLocal = %f, yLocal = %f",randomWaypointPosition[0], randomWaypointPosition[1]);
                numRandomWaypointSearchDistanceCriteriaFailed = 0;
                res.waypointList.at(numRandomWaypointsSelected-1).x = randomWaypointPosition[0];
                res.waypointList.at(numRandomWaypointsSelected-1).y = randomWaypointPosition[1];
                res.waypointList.at(numRandomWaypointsSelected-1).sampleProb = searchLocalMap.at(layerToString(_sampleProb), randomWaypointIndex);
                res.waypointList.at(numRandomWaypointsSelected-1).searchable = true;
                res.waypointList.at(numRandomWaypointsSelected-1).unskippable = false;
                res.waypointList.at(numRandomWaypointsSelected-1).maxAvoids = maxNormalWaypointAvoidCount;
                res.waypointList.at(numRandomWaypointsSelected-1).whiteProb = regionsOfInterest.at(searchLocalMapROINum).whiteProb;
                res.waypointList.at(numRandomWaypointsSelected-1).silverProb = regionsOfInterest.at(searchLocalMapROINum).silverProb;
                res.waypointList.at(numRandomWaypointsSelected-1).blueOrPurpleProb = regionsOfInterest.at(searchLocalMapROINum).blueOrPurpleProb;
                res.waypointList.at(numRandomWaypointsSelected-1).pinkProb = regionsOfInterest.at(searchLocalMapROINum).pinkProb;
                res.waypointList.at(numRandomWaypointsSelected-1).redProb = regionsOfInterest.at(searchLocalMapROINum).redProb;
                res.waypointList.at(numRandomWaypointsSelected-1).orangeProb = regionsOfInterest.at(searchLocalMapROINum).orangeProb;
                res.waypointList.at(numRandomWaypointsSelected-1).yellowProb = regionsOfInterest.at(searchLocalMapROINum).yellowProb;
            }
            if(numRandomWaypointSearchDistanceCriteriaFailed > randomWaypointDistanceCriteriaFailedLimit)
            {
                numRandomWaypointsToSelect--;
                //ROS_INFO("too many random waypoint distance criteria failed, numRandomWaypointToSelect = %i",numRandomWaypointsToSelect);
            }
        }
        if(res.waypointList.size() > numRandomWaypointsToSelect)
        {
            res.waypointList.erase(res.waypointList.end()-(res.waypointList.size()-numRandomWaypointsToSelect),res.waypointList.end());
        }
        //ROS_INFO("after selecting waypoints, vector size = %u",res.waypointList.size());
        for(int i=0; i<res.waypointList.size(); i++) // Transform random waypoint coordinates into global map coordinates
        {
            rotateCoord(res.waypointList.at(i).x, res.waypointList.at(i).y, res.waypointList.at(i).x, res.waypointList.at(i).y, -searchLocalMapHeading);
            res.waypointList.at(i).x += searchLocalMapXPos;
            res.waypointList.at(i).y += searchLocalMapYPos;
        }
        distanceMat.set_size(res.waypointList.size(),res.waypointList.size());
        for(int i=0; i<res.waypointList.size(); i++)
            for(int j=0; j<res.waypointList.size(); j++)
                distanceMat(i,j) = hypot(res.waypointList.at(i).x-res.waypointList.at(j).x, res.waypointList.at(i).y-res.waypointList.at(j).y);
        //ROS_INFO("distances:");
        //distanceMat.print();
#endif // GREEDY_SEARCH_WAYPOINT_SELECTION
    }
    else return false;
    return true;
}

bool MapManager::globalMapFullCallback(messages::GlobalMapFull::Request &req, messages::GlobalMapFull::Response &res)
{
    grid_map::GridMapRosConverter::toMessage(globalMap, res.globalMap);
    res.startingPlatformNum = startingPlatformLocation;
    return true;
}

bool MapManager::setStartingPlatformCallback(messages::SetStartingPlatform::Request &req, messages::SetStartingPlatform::Response &res)
{
    if(!req.responseOnly)
    {
        startingPlatformLocation = req.startingPlatformNum;
        if(req.fineAdjustment)
        {
            switch(startingPlatformLocation)
            {
            case 1:
                satMapStartE1Offset += req.deltaE;
                satMapStartS1Offset -= req.deltaN;
                break;
            case 2:
                satMapStartE2Offset += req.deltaE;
                satMapStartS2Offset -= req.deltaN;
                break;
            case 3:
                satMapStartE3Offset += req.deltaE;
                satMapStartS3Offset -= req.deltaN;
                break;
            default:
                satMapStartE2Offset += req.deltaE;
                satMapStartS2Offset -= req.deltaN;
                break;
            }
        }
        if(req.resetFineAdjustment)
        {
            switch(startingPlatformLocation)
            {
            case 1:
                satMapStartE1Offset = 0.0;
                satMapStartS1Offset = 0.0;
                break;
            case 2:
                satMapStartE2Offset = 0.0;
                satMapStartS2Offset = 0.0;
                break;
            case 3:
                satMapStartE3Offset = 0.0;
                satMapStartS3Offset = 0.0;
                break;
            default:
                satMapStartE2Offset = 0.0;
                satMapStartS2Offset = 0.0;
                break;
            }
        }
        setStartingPlatform();
        calculateGlobalMapSize();
        initializeGlobalMap();
        computeROIEastSouth();
        writeSatMapIntoGlobalMap();
        writeKeyframesIntoGlobalMap();
    }
    res.startingPlatformNum = startingPlatformLocation;
    return true;
}

void MapManager::keyframesCallback(const messages::KeyframeList::ConstPtr &msg) // tested
{
    keyframes = *msg;
    //writeSatMapIntoGlobalMap(); ?
    writeKeyframesIntoGlobalMap();
}

void MapManager::globalPoseCallback(const messages::RobotPose::ConstPtr &msg) // need to implement hsm to publish this
{
    globalPose = *msg;
    if(globalPose.northAngle != previousNorthAngle)
    {
        updateNorthTransformedMapData();
        previousNorthAngle = globalPose.northAngle;
        roisModifiedListMsg.ROIList = regionsOfInterest;
        roisModifiedPub.publish(roisModifiedListMsg);
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
    highestSampleValue = 0.0;
    cvSamplesFoundMsg = *msg;
    for(int i=0; i<cvSamplesFoundMsg.sampleList.size(); i++)
    {
        if(cvSamplesFoundMsg.sampleList.at(i).confidence > highestSampleValue) highestSampleValue = cvSamplesFoundMsg.sampleList.at(i).confidence;
    }
    if(searchLocalMapExists && (highestSampleValue < possibleSampleConfThresh))
    {
        rotateCoord(globalPose.x - searchLocalMapXPos, globalPose.y - searchLocalMapYPos, searchLocalMapRelPos[0], searchLocalMapRelPos[1], searchLocalMapHeading);
#ifdef USE_DONUT_SMASH
        donutSmash(searchLocalMap ,searchLocalMapRelPos);
#endif // USE_DONUT_SMASH
        grid_map::GridMapRosConverter::toMessage(searchLocalMap, searchLocalMapMsg);
        searchLocalMapPub.publish(searchLocalMapMsg);
        //addFoundSamples(grid_map::Position(keyframeRelPose.keyframeRelX, keyframeRelPose.keyframeRelY), keyframeRelPose.keyframeRelHeading);
    }
}

void MapManager::gridMapResetLayers(int startIndex, int endIndex, grid_map::GridMap &map) // tested
{
    for(int i=startIndex; i<=endIndex; i++)
    {
        if(static_cast<MAP_LAYERS_T>(i)==_satDriveability) map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), satDriveabilityInitialValue);
        else if(static_cast<MAP_LAYERS_T>(i)==_satDriveabilityConf) map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), satDriveabilityInitialConf);
        else if(static_cast<MAP_LAYERS_T>(i)==_keyframeDriveability) map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), keyframeDriveabilityInitialValue);
        else if(static_cast<MAP_LAYERS_T>(i)==_keyframeDriveabilityConf) map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), keyframeDriveabilityInitialConf);
        else map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), 0.0);
    }
}

void MapManager::gridMapAddLayers(int layerStartIndex, int layerEndIndex, grid_map::GridMap &map) // tested
{
    for(int i=layerStartIndex; i<=layerEndIndex; i++)
    {
        if(static_cast<MAP_LAYERS_T>(i)==_satDriveability) map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), satDriveabilityInitialValue);
        else if(static_cast<MAP_LAYERS_T>(i)==_satDriveabilityConf) map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), satDriveabilityInitialConf);
        else if(static_cast<MAP_LAYERS_T>(i)==_keyframeDriveability) map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), keyframeDriveabilityInitialValue);
        else if(static_cast<MAP_LAYERS_T>(i)==_keyframeDriveabilityConf) map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), keyframeDriveabilityInitialConf);
        else map.add(layerToString(static_cast<MAP_LAYERS_T>(i)), 0.0);
    }
}

void MapManager::donutSmash(grid_map::GridMap &map, grid_map::Position pos)
{
    float distanceToCell;
    float tp; // True positive rate
    float fn; // False negative rate
    float Pn1; // intermediate value based on law of total probability
    const float tn = 0.99; // True negative rate
    const float tpMaxValue = 0.99;
    const float tpMaxDistance = 5.0; // m
    float overallROIProb = 0.0;
    int temp;
    grid_map::Position cellPosition;
    donutSmashVerticies.clear();
    donutSmashVerticies.resize(4);
    donutSmashVerticies.at(0)[0] = pos[0] + donutSmashSearchRadius;
    donutSmashVerticies.at(0)[1] = pos[1] + donutSmashSearchRadius;
    donutSmashVerticies.at(1)[0] = pos[0] - donutSmashSearchRadius;
    donutSmashVerticies.at(1)[1] = pos[1] + donutSmashSearchRadius;
    donutSmashVerticies.at(2)[0] = pos[0] - donutSmashSearchRadius;
    donutSmashVerticies.at(2)[1] = pos[1] - donutSmashSearchRadius;
    donutSmashVerticies.at(3)[0] = pos[0] + donutSmashSearchRadius;
    donutSmashVerticies.at(3)[1] = pos[1] - donutSmashSearchRadius;
    //ROS_INFO("robot pos = [%f,%f]",pos[0],pos[1]);
    //ROS_INFO("donut verticies =\n[%f,%f]\n[%f,%f]\n[%f,%f]\n[%f,%f]",donutSmashVerticies.at(0)[0],donutSmashVerticies.at(0)[1],donutSmashVerticies.at(1)[0],donutSmashVerticies.at(1)[1],donutSmashVerticies.at(2)[0],donutSmashVerticies.at(2)[1],donutSmashVerticies.at(3)[0],donutSmashVerticies.at(3)[1]);
    donutSmashPolygon.removeVertices();
    donutSmashPolygon.addVertex(donutSmashVerticies.at(0));
    donutSmashPolygon.addVertex(donutSmashVerticies.at(1));
    donutSmashPolygon.addVertex(donutSmashVerticies.at(2));
    donutSmashPolygon.addVertex(donutSmashVerticies.at(3));
    map.getIndex(pos, donutSmashRobotPosIndex);
    for(grid_map::PolygonIterator donutIt(map, donutSmashPolygon); !donutIt.isPastEnd(); ++donutIt)
    {
        //ROS_INFO("donutIt = [%i,%i]",(*donutIt)[0],(*donutIt)[1]);
        if(!((*donutIt)[0] == donutSmashRobotPosIndex[0] && (*donutIt)[1] == donutSmashRobotPosIndex[1]))
        {
            map.getPosition(*donutIt, cellPosition);
            distanceToCell = hypot(cellPosition[0] - pos[0], cellPosition[1] - pos[1]);
            //ROS_INFO("distanceToCell = %f",distanceToCell);
            tp = tpMaxValue - pow(distanceToCell/tpMaxDistance, 2.0);
            if(tp<0.0) tp = 0.0;
            //ROS_INFO("tp = %f",tp);
            fn = 1.0 - tp;
            //ROS_INFO("fn = %f",fn);
            Pn1 = fn*map.at(layerToString(_sampleProb), *donutIt) + tn*(1.0 - map.at(layerToString(_sampleProb), *donutIt));
            //ROS_INFO("Pn1 = %f",Pn1);
            //ROS_INFO("old donut cell value [%i,%i] = %f",(*donutIt)[0],(*donutIt)[1],map.at(layerToString(_sampleProb), *donutIt));
            map.at(layerToString(_sampleProb), *donutIt) = fn*map.at(layerToString(_sampleProb), *donutIt)/Pn1;
            //ROS_INFO("new donut cell value, before coersion [%i,%i] = %f",(*donutIt)[0],(*donutIt)[1],map.at(layerToString(_sampleProb), *donutIt));
            if(map.at(layerToString(_sampleProb), *donutIt) > 1.0) map.at(layerToString(_sampleProb), *donutIt) = 1.0;
            else if(map.at(layerToString(_sampleProb), *donutIt) < 0.0) map.at(layerToString(_sampleProb), *donutIt) = 0.0;
            //ROS_INFO("new donut cell value, after coersion [%i,%i] = %f",(*donutIt)[0],(*donutIt)[1],map.at(layerToString(_sampleProb), *donutIt));
            overallROIProb = 0.0;
            for(grid_map::GridMapIterator wholeIt(map); !wholeIt.isPastEnd(); ++wholeIt)
            {
                if(!((*wholeIt)[0] == (*donutIt)[0] && (*wholeIt)[1] == (*donutIt)[1]))
                {
                    //ROS_INFO("old other cell value [%i,%i] = %f",(*wholeIt)[0],(*wholeIt)[1],map.at(layerToString(_sampleProb), *wholeIt));
                    map.at(layerToString(_sampleProb), *wholeIt) = tn*map.at(layerToString(_sampleProb), *wholeIt)/Pn1;

                    if(map.at(layerToString(_sampleProb), *wholeIt) >= 1.0)
                    {
                        //ROS_WARN("CELL SET TO >= 1");
                        //ROS_INFO("new other cell value before coersion [%i,%i] = %f",(*wholeIt)[0],(*wholeIt)[1],map.at(layerToString(_sampleProb), *wholeIt));
                        //ROS_INFO("distanceToCell = %f",distanceToCell);
                        //ROS_INFO("tp = %f",tp);
                        //ROS_INFO("fn = %f",fn);
                        //ROS_INFO("Pn1 = %f",Pn1);
                        map.at(layerToString(_sampleProb), *wholeIt) = 1.0;
                        //std::cout << "enter a character to continue" << std::endl;
                        //std::cin >> temp;
                    }
                    else if(map.at(layerToString(_sampleProb), *wholeIt) < 0.0) map.at(layerToString(_sampleProb), *wholeIt) = 0.0;
                    //ROS_INFO("new other cell value after coersion [%i,%i] = %f",(*wholeIt)[0],(*wholeIt)[1],map.at(layerToString(_sampleProb), *wholeIt));
                }
                overallROIProb += map.at(layerToString(_sampleProb), *wholeIt);
            }
        }
    }
    regionsOfInterest.at(searchLocalMapROINum).sampleProb = overallROIProb;
    roisModifiedListMsg.ROIList = regionsOfInterest;
    roisModifiedPub.publish(roisModifiedListMsg);
    ROS_INFO("overall ROI prob after smash = %f",overallROIProb);
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
    int i,j;
    gridMapResetLayers((int)_slope, (int)_satObjectHeight, globalMap);
    /*// Slope
    for(grid_map::GridMapIterator it(globalMap); !it.isPastEnd(); ++it)
    {
        globalMap.getPosition(*it,globalMapToSatMapPos);
        rotateCoord(globalMapToSatMapPos[0], globalMapToSatMapPos[1], globalMapToSatMapPos[0], globalMapToSatMapPos[1], -(globalPose.northAngle-90.0));
        globalMapToSatMapPos[0] += satMapStartE;
        globalMapToSatMapPos[1] += satMapStartS;
        j = (int)(round(globalMapToSatMapPos[0]-slopeMapRes/2.0)/slopeMapRes);
        i = (int)(round(globalMapToSatMapPos[1]-slopeMapRes/2.0)/slopeMapRes);
        if(j>=0 && j<slopeNumCols && i>=0 && i<slopeNumRows) globalMap.at(layerToString(_slope),*it) = (float)slopeMap[i][j];
    }*/
    /*for(int i=0; i<slopeNumRows; i++)
    {
        for(int j=0; j<slopeNumCols; j++)
        {
            rotateCoord((float)(j*slopeMapRes+slopeMapRes/2.0) - satMapStartE, (float)(i*slopeMapRes+slopeMapRes/2.0) - satMapStartS, satMapToGlobalMapPos[0], satMapToGlobalMapPos[1], globalPose.northAngle-90.0);
            //satMapToGlobalMapPos[0] = (float)(j*slopeMapRes+slopeMapRes/2.0) - satMapStartE;
            //satMapToGlobalMapPos[1] = (float)(i*slopeMapRes+slopeMapRes/2.0) - satMapStartS;
            if(globalMap.isInside(satMapToGlobalMapPos)) globalMap.atPosition(layerToString(_slope), satMapToGlobalMapPos) = slopeMap[i][j]; // *** Got to figure out if this is correct
        }
    }*/
    // Driveability
    for(grid_map::GridMapIterator it(globalMap); !it.isPastEnd(); ++it)
    {
        globalMap.getPosition(*it,globalMapToSatMapPos);
        rotateCoord(globalMapToSatMapPos[0], globalMapToSatMapPos[1], globalMapToSatMapPos[0], globalMapToSatMapPos[1], -(globalPose.northAngle-90.0));
        globalMapToSatMapPos[0] += satMapStartE;
        globalMapToSatMapPos[1] += satMapStartS;
        j = (int)(round(globalMapToSatMapPos[0]-driveabilityMapRes/2.0)/driveabilityMapRes);
        i = (int)(round(globalMapToSatMapPos[1]-driveabilityMapRes/2.0)/driveabilityMapRes);
        if(j>=0 && j<driveabilityNumCols && i>=0 && i<driveabilityNumRows)
        {
            globalMap.at(layerToString(_satDriveability),*it) = (float)driveabilityMap[i][j];
        }
    }
    /*for(int i=0; i<driveabilityNumRows; i++)
    {
        for(int j=0; j<driveabilityNumCols; j++)
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
    }*/
    /*grid_map::GridMapRosConverter::toMessage(globalMap, globalMapMsg);
    globalMapPub.publish(globalMapMsg);
    ros::Duration(3).sleep();*/
    //smoothDriveabilityLayer();
    grid_map::GridMapRosConverter::toMessage(globalMap, globalMapMsg);
    globalMapPub.publish(globalMapMsg);
}

void MapManager::writeKeyframesIntoGlobalMap()
{
    ++keyframeWriteIntoGlobalMapSerialNum;
    gridMapResetLayers(MAP_KEYFRAME_LAYERS_START_INDEX, MAP_KEYFRAME_LAYERS_END_INDEX, globalMap);
    for(int i=0; i<keyframes.keyframeList.size(); i++)
    {
        grid_map::GridMapRosConverter::fromMessage(keyframes.keyframeList.at(i).map,currentKeyframe);
        keyframeHeading = keyframes.keyframeList.at(i).heading;
        keyframeXPos = keyframes.keyframeList.at(i).x;
        keyframeYPos = keyframes.keyframeList.at(i).y;
        //ROS_INFO("currentKeyframe.at = %f",currentKeyframe.atPosition(layerToString(_keyframeDriveability), grid_map::Position(30.0,30.0)));
        for(grid_map::GridMapIterator it(globalMap); !it.isPastEnd(); ++it)
        {
            globalMap.getPosition(*it, globalTransformCoord);
            rotateCoord(globalTransformCoord[0]-keyframeXPos, globalTransformCoord[1]-keyframeYPos, keyframeCoord[0], keyframeCoord[1], keyframeHeading);
            if(currentKeyframe.isInside(keyframeCoord))
            {
                if(currentKeyframe.atPosition(layerToString(_keyframeDriveability), keyframeCoord)!=0.0)
            	   globalMap.at(layerToString(_keyframeDriveability), *it) = currentKeyframe.atPosition(layerToString(_keyframeDriveability), keyframeCoord);
                /*// Driveability, confidence, and height
                if(currentKeyframe.atPosition(layerToString(_keyframeDriveabilityConf),keyframeCoord)>globalMap.at(layerToString(_keyframeDriveabilityConf),*it))
                {
                    globalMap.at(layerToString(_keyframeDriveability), *it) = currentKeyframe.atPosition(layerToString(_keyframeDriveability), keyframeCoord);
                    //globalMap.at(layerToString(_keyframeDriveabilityConf), *it) = currentKeyframe.atPosition(layerToString(_keyframeDriveabilityConf), keyframeCoord);
                    //globalMap.at(layerToString(_keyframeObjectHeight), *it) = currentKeyframe.atPosition(layerToString(_keyframeObjectHeight), keyframeCoord);
                }
                else if(currentKeyframe.atPosition(layerToString(_keyframeDriveabilityConf),keyframeCoord)==globalMap.at(layerToString(_keyframeDriveabilityConf),*it))
                {
                    if(currentKeyframe.atPosition(layerToString(_keyframeDriveability), keyframeCoord)>globalMap.at(layerToString(_keyframeDriveability), *it))
                    {
                        globalMap.at(layerToString(_keyframeDriveability), *it) = currentKeyframe.atPosition(layerToString(_keyframeDriveability), keyframeCoord);
                        globalMap.at(layerToString(_keyframeDriveabilityConf), *it) = currentKeyframe.atPosition(layerToString(_keyframeDriveabilityConf), keyframeCoord);
                        globalMap.at(layerToString(_keyframeObjectHeight), *it) = currentKeyframe.atPosition(layerToString(_keyframeObjectHeight), keyframeCoord);
                    }
                }*/
                // Reflectivity
                //globalMap.at(layerToString(_reflectivity), *it) = currentKeyframe.atPosition(layerToString(_reflectivity), keyframeCoord);
                /*for(int j=MAP_KEYFRAME_LAYERS_START_INDEX; j<=MAP_KEYFRAME_LAYERS_END_INDEX; j++)
                {
                    currentCellValue = globalMap.at(layerToString(static_cast<MAP_LAYERS_T>(j)), *it);
                    possibleNewCellValue = currentKeyframe.atPosition(layerToString(static_cast<MAP_LAYERS_T>(j)), keyframeCoord);
                    if((static_cast<MAP_LAYERS_T>(j)==_keyframeDriveability) || (static_cast<MAP_LAYERS_T>(j)==_reflectivity))
                    {
                        if((possibleNewCellValue > currentCellValue) || (globalMap.at(layerToString(_keyframeWriteIntoGlobalMapSerialNum), *it) != (float)keyframeWriteIntoGlobalMapSerialNum))
                            globalMap.at(layerToString(static_cast<MAP_LAYERS_T>(j)), *it) = possibleNewCellValue;
                    }
                    else if(static_cast<MAP_LAYERS_T>(j)==_objectHeight)
                    {
                        if(currentKeyframe.atPosition(layerToString(_driveability), keyframeCoord) == _noObject) globalMap.at(layerToString(static_cast<MAP_LAYERS_T>(j)), *it) = 0;
                        else
                        {
                            if((possibleNewCellValue < currentCellValue) || (globalMap.at(layerToString(_keyframeWriteIntoGlobalMapSerialNum), *it) != (float)keyframeWriteIntoGlobalMapSerialNum))
                                globalMap.at(layerToString(static_cast<MAP_LAYERS_T>(j)), *it) = possibleNewCellValue;
                        }
                    }
                }
                globalMap.at(layerToString(_keyframeWriteIntoGlobalMapSerialNum), *it) = (float)keyframeWriteIntoGlobalMapSerialNum;*/
            }
        }
        /*for(grid_map::GridMapIterator it(currentKeyframe); !it.isPastEnd(); ++it)
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
        }*/
    }
    //ROS_INFO("globalMap.at = %f",globalMap.atPosition(layerToString(_driveability), grid_map::Position(50.0,45.0)));
    grid_map::GridMapRosConverter::toMessage(globalMap, globalMapMsg);
    globalMapPub.publish(globalMapMsg);
}

void MapManager::northTransformROIs() // tested
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

/*void MapManager::smoothDriveabilityLayer() // tested
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
}*/

void MapManager::setStartingPlatform()
{
    switch(startingPlatformLocation)
    {
    case 1:
        satMapStartE = satMapStartE1 + satMapStartE1Offset;
        satMapStartS = satMapStartS1 + satMapStartS1Offset;
        break;
    case 2:
        satMapStartE = satMapStartE2 + satMapStartE2Offset;
        satMapStartS = satMapStartS2 + satMapStartS2Offset;
        break;
    case 3:
        satMapStartE = satMapStartE3 + satMapStartE3Offset;
        satMapStartS = satMapStartS3 + satMapStartS3Offset;
        break;
    default:
        satMapStartE = satMapStartE2 + satMapStartE2Offset;
        satMapStartS = satMapStartS2 + satMapStartS2Offset;
        break;
    }
}

void MapManager::calculateGlobalMapSize()
{
    float candidateSize;
    float bestCandidateSize = 0.0;
    candidateSize = hypot(satMapStartE, satMapStartS);
    //ROS_INFO("candidateSize = %f",candidateSize);
    //ROS_INFO("bestCandidateSize = %f",bestCandidateSize);
    if(candidateSize>bestCandidateSize) bestCandidateSize = candidateSize;
    candidateSize = hypot(satMapStartE, satMapSize[1] - satMapStartS);
    //ROS_INFO("candidateSize = %f",candidateSize);
    //ROS_INFO("bestCandidateSize = %f",bestCandidateSize);
    if(candidateSize>bestCandidateSize) bestCandidateSize = candidateSize;
    candidateSize = hypot(satMapSize[0] - satMapStartE, satMapStartS);
    //ROS_INFO("candidateSize = %f",candidateSize);
    //ROS_INFO("bestCandidateSize = %f",bestCandidateSize);
    if(candidateSize>bestCandidateSize) bestCandidateSize = candidateSize;
    candidateSize = hypot(satMapSize[0] - satMapStartE, satMapSize[1] - satMapStartS);
    //ROS_INFO("candidateSize = %f",candidateSize);
    //ROS_INFO("bestCandidateSize = %f",bestCandidateSize);
    if(candidateSize>bestCandidateSize) bestCandidateSize = candidateSize;
    //ROS_INFO("candidateSize = %f",candidateSize);
    //ROS_INFO("bestCandidateSize = %f",bestCandidateSize);
    globalMapSize[0] = bestCandidateSize*2.0;
    globalMapSize[1] = bestCandidateSize*2.0;
    //ROS_INFO("globalMapSize[0] = %f, [1] = %f",globalMapSize[0],globalMapSize[1]);
}

void MapManager::initializeGlobalMap()
{
    globalMap.setGeometry(globalMapSize, mapResolution, globalMapOrigin);
    globalMapTemp.setGeometry(globalMapSize, mapResolution, globalMapOrigin);
    globalMapRowsCols = globalMap.getSize();
    gridMapAddLayers(0, NUM_MAP_LAYERS-1, globalMap);
    gridMapAddLayers(0, NUM_MAP_LAYERS-1, globalMapTemp);
}

void MapManager::cutOutGlobalSubMap()
{

}

void MapManager::computeROIEastSouth()
{
	for(int i=0; i<regionsOfInterest.size(); i++)
	{
		regionsOfInterest.at(i).e = regionsOfInterest.at(i).eMap - satMapStartE;
		regionsOfInterest.at(i).s = regionsOfInterest.at(i).sMap - satMapStartS;
	}
}
