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
	std::vector<float> value;
	float computedValue;
	float valueSum;
	std::vector<float> valueNormalized;
	float valueNormalizedSum;
	float valueNormalizedFloor;
	float randomValue;
	float largestNormalizedValue;
	std::vector<int> notVisited;
	int notVisitedSum;
	arma::Mat<float> pheromone;
	arma::Mat<float> distance;
	arma::Mat<float> terrainHazard;
	const float sampleProbGain = 1.0;
	const float pheromoneGain = 0.5;
	const float distanceGain = 0.5;
	const float terrainGain = 0.7;
	const float pheroDepoGain = 0.8;
	const float pheroDecayValue = 0.01;
	const float initialPheromone = 0.5;
	const int maxAntNum = 100;
	int antNum;
	int i;
	int j;
	int bestJ;
	int numWaypointsToPlan;
	std::vector<robot_control::Waypoint> waypointsToPlan;
	robot_control::Waypoint currentLocation;
	float bestPheromone;
	// Methods
	void roiTimeExpiredCallback_();
	void chooseRandomWaypoints_();
	void antColony_();
};

#endif // SEARCH_REGION_H
