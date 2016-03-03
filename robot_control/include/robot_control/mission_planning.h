#ifndef MISSION_PLANNING_H
#define MISSION_PLANNING_H
#include <ros/ros.h>
#include <robot_control/Waypoint.h>
#include <robot_control/WaypointsOfInterest.h> // Remove
#include <robot_control/RegionsOfInterest.h>
#include <robot_control/WaypointsOfRegion.h>
#include <robot_control/IntermediateWaypoints.h>
#include "robot_status.h"
#include "action_type_enum.h"
#include <messages/ExecAction.h>
#include <messages/NavFilterOut.h>
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
	ros::ServiceClient execActionClient;
	messages::ExecAction execActionSrv;
	ros::Subscriber navSub;
	ros::ServiceClient intermediateWaypointsClient;
	robot_control::IntermediateWaypoints intermediateWaypointsSrv;
	const int loopRate = 20; // Hz
	std::vector<int> value;
	int computedValue;
	int valueSum;
	std::vector<int> valueNormalized;
	int valueNormalizedSum;
	int valueNormalizedFloor;
	int randomValue;
	int largestNormalizedValue;
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
	const int pheroDepoGain = 800;
	const int pheroDecayValue = 10;
	const int initialPheromone = 500;
	const int maxAntNum = 100;
	int antNum;
	int i;
	int j;
	int bestJ;
	robot_control::Waypoint currentLocation;
	int numWaypointsToPlan;
	int numWaypointsToTravel;
	std::vector<robot_control::Waypoint> waypointsToPlan;
	std::vector<robot_control::Waypoint> waypointsToTravel;
	int bestPheromone;
	RobotStatus robotStatus;
	bool collisionDetected;
	bool commandedAvoidObstacle;
	bool possessingSample;
	bool commandedReturnHome;
	bool confirmedApproach;
	bool commandedApproach;
	bool interestingSampleNearby;
	bool commandedExamine;
	bool inIncompleteROI;
	bool commandedPlanRegionPath;
	bool completedROI;
	bool completedDeposit;
	bool commandedChooseRegion;
	bool initComplete;
	bool commandedInit;
	const float homeX = 5.0; // m
	const float homeY = 0.0; // m
private:
	void avoidObstacle_();
	void returnHome_();
	void approach_();
	void examine_();
	void planRegionPath_();
	void chooseRegion_();
	void init_();
	void sendDriveGlobal_();
	void antColony_();
	void planSafePath_();
	void clearAndResizeWTT_();
	void callIntermediateWaypoints_();
	void navCallback_(const messages::NavFilterOut::ConstPtr& msg);
};

#endif // MISSION_PLANNING_H
