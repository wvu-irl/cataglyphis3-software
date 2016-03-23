#ifndef MISSION_PLANNING_H
#define MISSION_PLANNING_H
#include <ros/ros.h>
#include "mission_planning_process_share.h"
#include <messages/NavFilterOut.h>
#include <messages/ExecActionEnded.h>
#include <messages/nb1_to_i7_msg.h>
#include "avoid.h"
#include "next_best_region.h"
//#include "search_closest_region.h"
//#include "examine.h"
#include "approach.h"
#include "collect.h"
#include "confirm_collect.h"
#include "go_home.h"
#include "deposit_approach.h"
#include "deposit_sample.h"
#include "pause.h"
#include "bit_utils.h"

class MissionPlanning : public MissionPlanningProcessShare
{
public:
	// Methods
	MissionPlanning();
	void run();
	// Members
	ros::NodeHandle nh;
	ros::Subscriber navSub;
	ros::Subscriber ExecActionEndedSub;
    ros::Subscriber nb1Sub;
	ros::Subscriber collisionSub;
    messages::nb1_to_i7_msg nb1Msg;
	const int loopRate = 20; // Hz
	Avoid avoid;
	NextBestRegion nextBestRegion;
	Approach approach;
	Collect collect;
	ConfirmCollect confirmCollect;
	GoHome goHome;
	DepositApproach depositApproach;
	DepositSample depositSample;
    Pause pause;
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
	bool collisionInterruptTrigger;
	Leading_Edge_Latch collisionInterruptLEL;
	unsigned int numProcsToExec;
	bool multiProcLockout;
	unsigned int lockoutSum;
	bool initComplete;
	bool pauseStarted;
	const float homeX = 5.0; // m
	const float homeY = 0.0; // m
	const float collisionDistanceThresh = 5.0; // m
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
	void runPause_();
	void antColony_();
	void calcNumProcsToExec_();
	void findBestSample();

	void navCallback_(const messages::NavFilterOut::ConstPtr& msg);
	void ExecActionEndedCallback_(const messages::ExecActionEnded::ConstPtr& msg);
    void nb1Callback_(const messages::nb1_to_i7_msg::ConstPtr& msg);
	void collisionCallback_(const messages::CollisionOut::ConstPtr& msg);
	void execInfoCallback_(const messages::ExecInfo::ConstPtr& msg);
	void cvSamplesCallback_(const messages::CVSamplesFound::ConstPtr& msg);
};

#endif // MISSION_PLANNING_H
