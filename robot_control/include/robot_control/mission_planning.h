#ifndef MISSION_PLANNING_H
#define MISSION_PLANNING_H
#include <ros/ros.h>
#include <robot_control/Waypoint.h>
#include <robot_control/WaypointsOfInterest.h>
#include <messages/ExecAction.h>
#include <vector>
#include <armadillo>
#include <math.h>

class MissionPlanning
{
public:
	// Methods
	MissionPlanning();
	void run();
	// Members
	ros::NodeHandle nh;
	ros::ServiceClient woiClient;
	robot_control::WaypointsOfInterest woiSrv;
	ros::Publisher execActionPub;
	messages::ExecAction execActionMsg;
	std::vector<int> value;
	int valueSum;
	std::vector<int> valueNormalized;
	int valueNormalizedFloor;
	int randomValue;
	std::vector<int> notVisited;
	int notVisitedSum;
	arma::Mat<int> pheromone;
	arma::Mat<int> distance;
	arma::Mat<int> terrainHazard;
	const int easyProbGain = 750;
	int includeEasy;
	const int medProbGain = 850;
	int includeMed;
	const int hardProbGain = 950;
	int includeHard;
	const int pheromoneGain = 500;
	const int distanceGain = 500;
	const int terrainGain = 700;
	const int pheroDepoGain = 500;
	const int pheroDecayValue = 100;
	const int initialPheromone = 500;
	const int maxAntNum = 1000;
	int antNum;
	int i;
	int j;
	int bestJ;
	robot_control::Waypoint currentLocation;
	int numInterestingWaypoints;
	std::vector<robot_control::Waypoint> waypointsToTravel;
	int bestPheromone;
private:
	void planNewPath_();
	void antColony_();
};

#endif // MISSION_PLANNING_H
