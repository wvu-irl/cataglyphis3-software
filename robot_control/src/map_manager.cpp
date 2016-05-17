#include <robot_control/map_manager.h>

MapManager::MapManager()
    : satMap({"sampleProb","hazard"})
{
    roiServ = nh.advertiseService("/control/mapmanager/regionsofinterest", &MapManager::listROI, this);
    modROIServ = nh.advertiseService("/control/mapmanager/modifyroi", &MapManager::modROI, this);
    mapROIServ = nh.advertiseService("/control/mapmanager/roigridmap", &MapManager::mapROI, this);
    keyframesSub = nh.subscribe<messages::KeyframeList>("/slam/slam/keyframelist", 1, &MapManager::keyframesCallback, this);
    currentROI = 0; // 0 means in no ROI

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
    satMapPub = nh.advertise<grid_map_msgs::GridMap>("control/mapmanager/satmap",1);
    satMap.setFrameId("map");
    satMap.setGeometry(grid_map::Length(300.0, 200.0), 1.0, grid_map::Position(12.0, 65.0));
    satMap.add("sampleProb", 458);
    satMap.add("hazard", 998);
    /*for(grid_map::GridMapIterator it(satMap); !it.isPastEnd(); ++it)
    {
        satMap.at("sampleProb", *it) = 322.9;
        satMap.at("hazard", *it) = 190.1;
    }*/
    satMap.atPosition("sampleProb", grid_map::Position(50.1, 7.0)) = 500.2;
    ROS_INFO("prob at (80,80) = %f", satMap.atPosition("sampleProb", grid_map::Position(80.0, 80.0)));
    ROS_INFO("prob at (90,7.2) = %f", satMap.atPosition("sampleProb", grid_map::Position(90.0, 7.2)));
    ROS_INFO("prob at (90.1,7.0) = %f", satMap.atPosition("sampleProb", grid_map::Position(90.1, 7.0)));
    ROS_INFO("prob at (90.5,7.0) = %f", satMap.atPosition("sampleProb", grid_map::Position(90.5, 7.0)));
    ROS_INFO("prob at (90.5,6.8) = %f", satMap.atPosition("sampleProb", grid_map::Position(90.5, 6.8)));
    grid_map::GridMapRosConverter::toMessage(satMap,satMapMsg);
    ros::Duration(1.0).sleep();
    satMapPub.publish(satMapMsg);
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

void MapManager::keyframesCallback(const messages::KeyframeList::ConstPtr &msg)
{
    keyframes = *msg;
    updateCurrentROI();
}

void MapManager::updateCurrentROI()
{

}
