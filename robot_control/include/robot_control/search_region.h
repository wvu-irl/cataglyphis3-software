#ifndef SEARCH_REGION_H
#define SEARCH_REGION_H
#include "procedure.h"

class SearchRegion : public Procedure
{
public:
	// Members
	ros::NodeHandle nh;
	bool roiTimeExpired;
	bool allWaypointsVisited;
	ros::Timer roiTimer;
	const int numRandomWaypoints = 5;
	// Methods
	bool runProc();
	SearchRegion(); // constructor
private:
	// Members
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
	int numWaypointsToPlan;
	std::vector<robot_control::Waypoint> waypointsToPlan;
	int bestPheromone;
	// Methods
	void roiTimeExpiredCallback_();
	void chooseRandomWaypoints_();
	void antColony_();
};

#endif // SEARCH_REGION_H
