/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef SEARCH_REGION_H
#define SEARCH_REGION_H
#include "procedure.h"

class SearchRegion : public Procedure
{
public:
	// Members
	ros::NodeHandle nh;
	bool allWaypointsVisited;
	const int numRandomWaypoints = 5;
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
