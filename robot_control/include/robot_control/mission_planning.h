#ifndef MISSION_PLANNING_H
#define MISSION_PLANNING_H
#include <ros/ros.h>
#include "mission_planning_process_share.h"
#include <messages/NavFilterOut.h>
#include <messages/ExecDequeEmpty.h>
#include "choose_region.h"


class MissionPlanning : public MissionPlanningProcessShare
{
public:
	// Methods
	MissionPlanning();
	void run();
	// Members
	ros::NodeHandle nh;
	ros::Subscriber navSub;
        ros::Subscriber execDequeEmptySub;
	const int loopRate = 20; // Hz
        ChooseRegion chooseRegion;
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


	std::vector<robot_control::Waypoint> waypointsToPlan;
	int bestPheromone;
	bool collisionDetected;
	bool commandedAvoidObstacle;
	bool possessingSample;
	bool commandedReturnHome;
	bool confirmedAcquire;
	bool commandedAcquire;
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
        void avoidObstacle_(); // ***
        void returnHome_();// ***
        void deposit_();// ***
        void acquire_();// ***
        void examine_();// ***
        void planRegionPath_();// ***
        void chooseRegion_();// ***
        void init_();// ***
        void evalConditions_();
        void runProcesses_();

	void antColony_();

	void navCallback_(const messages::NavFilterOut::ConstPtr& msg);
        void execDequeEmptyCallback_(const messages::ExecDequeEmpty::ConstPtr& msg);
};

#endif // MISSION_PLANNING_H
