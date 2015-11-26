#include <robot_control/map_manager.h>

MapManager::MapManager()
{
	woiServ = nh.advertiseService("/control/mapmanager/waypointsofinterest", &MapManager::WOI, this);
	// *** Temporary! Only for Testing ***
	waypoint.x = 87.6;
	waypoint.y = 11.2;
	waypoint.easyProb = 500;
	waypoint.medProb = 200;
	waypoint.hardProb = 44;
	waypoint.terrainHazard = 35;
	waypointsOfInterestVector.push_back(waypoint);
	waypoint.x = 100.1;
	waypoint.y = 33.9;
	waypoint.easyProb = 5;
	waypoint.medProb = 2;
	waypoint.hardProb = 9;
	waypoint.terrainHazard = 55;
	waypointsOfInterestVector.push_back(waypoint);
	waypoint.x = 150.0;
	waypoint.y = 2.6;
	waypoint.easyProb = 880;
	waypoint.medProb = 800;
	waypoint.hardProb = 4;
	waypoint.terrainHazard = 305;
	waypointsOfInterestVector.push_back(waypoint);
	waypoint.x = 77.1;
	waypoint.y = 15.2;
	waypoint.easyProb = 50;
	waypoint.medProb = 600;
	waypoint.hardProb = 84;
	waypoint.terrainHazard = 500;
	waypointsOfInterestVector.push_back(waypoint);
	waypoint.x = 0.6;
	waypoint.y = -51.2;
	waypoint.easyProb = 60;
	waypoint.medProb = 20;
	waypoint.hardProb = 440;
	waypoint.terrainHazard = 9;
	waypointsOfInterestVector.push_back(waypoint);
	waypoint.x = 97.6;
	waypoint.y = 1.2;
	waypoint.easyProb = 900;
	waypoint.medProb = 20;
	waypoint.hardProb = 4;
	waypoint.terrainHazard = 100;
	waypointsOfInterestVector.push_back(waypoint);
	waypoint.x = 200.1;
	waypoint.y = 60.2;
	waypoint.easyProb = 1;
	waypoint.medProb = 21;
	waypoint.hardProb = 7;
	waypoint.terrainHazard = 600;
	waypointsOfInterestVector.push_back(waypoint);
	waypoint.x = 52.2;
	waypoint.y = 19.3;
	waypoint.easyProb = 700;
	waypoint.medProb = 60;
	waypoint.hardProb = 99;
	waypoint.terrainHazard = 10;
	waypointsOfInterestVector.push_back(waypoint);
	waypoint.x = -100.6;
	waypoint.y = -101.2;
	waypoint.easyProb = 80;
	waypoint.medProb = 20;
	waypoint.hardProb = 25;
	waypoint.terrainHazard = 700;
	waypointsOfInterestVector.push_back(waypoint);
	waypoint.x = -87.6;
	waypoint.y = -11.2;
	waypoint.easyProb = 900;
	waypoint.medProb = 40;
	waypoint.hardProb = 44;
	waypoint.terrainHazard = 100;
	waypointsOfInterestVector.push_back(waypoint);
	ROS_INFO("wapointsOfInterestVector Size = %u",waypointsOfInterestVector.size());
	// ***********************************

}

bool MapManager::WOI(robot_control::WaypointsOfInterest::Request &req, robot_control::WaypointsOfInterest::Response &res)
{
	res.waypointArray = waypointsOfInterestVector;
	ROS_INFO("sent WOI");
	return true;
}
