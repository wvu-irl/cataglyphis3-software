#ifndef SEARCH_REGION_H
#define SEARCH_REGION_H
#include "procedure.h"

//#define USE_DONUT_SMASH

class SearchRegion : public Procedure
{
public:
	// Members
	ros::NodeHandle nh;
	bool allWaypointsVisited;
#ifdef USE_DONUT_SMASH
	const int numRandomWaypoints = 1;
#else
	const int numRandomWaypoints = 5;
#endif // USE_DONUT_SMASH
	// Methods
	bool runProc();
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
	const float hazardCorridorWidth = 2.0; // m
	const float sampleProbGain = 1.0;
	const float pheromoneGain = 0.5;
	const float distanceGain = 0.05;
	const float terrainGain = 0.2;
	const float pheroDepoGain = 0.8;
	const float pheroDecayValue = 0.01;
	const float initialPheromone = 0.5;
	const float maxPheromoneValue = 1.0;
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
	void chooseRandomWaypoints_();
	void antColony_();
};

#endif // SEARCH_REGION_H
