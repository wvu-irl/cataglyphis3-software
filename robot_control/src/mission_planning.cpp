#include <robot_control/mission_planning.h>

MissionPlanning::MissionPlanning()
{
	woiClient = nh.serviceClient<robot_control::WaypointsOfInterest>("/control/mapmanager/waypointsofinterest");
	woiSrv.request.easyThresh = 0;
	woiSrv.request.medThresh = 0;
	woiSrv.request.hardThresh = 0;
}

void MissionPlanning::run()
{
	if(woiClient.call(woiSrv)) ROS_INFO("woi service call successful");
	else ROS_ERROR("woi service call unsuccessful");
	currentLocation.x = 0.0; // Replace with actual current x and y
	currentLocation.y = 0.0; // ***
	woiSrv.response.waypointArray.push_back(currentLocation);
	numInterestingWaypoints = woiSrv.response.waypointArray.size();
	value.resize(numInterestingWaypoints);
	valueNormalized.resize(numInterestingWaypoints);
	pheromone.set_size(numInterestingWaypoints,numInterestingWaypoints);
	pheromone.fill(initialPheromone);
	distance.set_size(numInterestingWaypoints,numInterestingWaypoints);
	terrainHazard.set_size(numInterestingWaypoints,numInterestingWaypoints);
	for(int m=0; m<numInterestingWaypoints; m++)
	{
		for(int n=0; n<numInterestingWaypoints; n++)
		{
			distance(m,n) = hypot(woiSrv.response.waypointArray.at(m).x - woiSrv.response.waypointArray.at(n).x,
								  woiSrv.response.waypointArray.at(m).y - woiSrv.response.waypointArray.at(n).y);
			// terrainHazard(m,n) = something;
		}
	}
	antNum = 0;
	i = 0;
	j = 0;
	for(antNum; antNum<maxAntNum; antNum++)
	{
		notVisited.clear();
		notVisited.resize(numInterestingWaypoints,1);
		notVisited.back() = 0; // Do not want to include current location in possible set of waypoints to visit
		notVisitedSum = numInterestingWaypoints;
		while(notVisitedSum!=0)
		{
			valueSum = 0;
			valueNormalizedFloor = 0;
			for(j; j<numInterestingWaypoints; j++)
			{
				value.at(j) = notVisited.at(j)*(easyProbGain*woiSrv.response.waypointArray.at(j).easyProb +
												medProbGain*woiSrv.response.waypointArray.at(j).medProb +
												hardProbGain*woiSrv.response.waypointArray.at(j).hardProb +
												pheromoneGain*pheromone(i,j) -
												distanceGain*distance(i,j) -
												terrainGain*terrainHazard(i,j))/(easyProbGain+medProbGain+hardProbGain+pheromoneGain);
			}
			for(int k=0; k<numInterestingWaypoints; k++) valueSum += value.at(k);
			for(int k=0; k<numInterestingWaypoints; k++) valueNormalized.at(k) = value.at(k)/valueSum;
			randomValue = rand() % 1001;
			for(int k=0; k<numInterestingWaypoints; k++)
			{
				if(randomValue >= valueNormalizedFloor && randomValue <= (valueNormalizedFloor + valueNormalized.at(k))) {bestJ = k; break;}
				else valueNormalizedFloor += valueNormalized.at(k);
			}
			pheromone(i,bestJ) += pheroDepoGain/distance(i,bestJ);
			pheromone(bestJ,i) += pheroDepoGain/distance(bestJ,i);
			notVisited.at(bestJ) = 0;
			i = bestJ;
			for(int k=0; k<numInterestingWaypoints; k++) notVisitedSum += notVisited.at(k);
		}
		for(int m=0; m<numInterestingWaypoints; m++)
		{
			for(int n=0; n<numInterestingWaypoints; n++)
			{
				pheromone(m,n) -= pheroDecayValue;
				if(pheromone(m,n)<0) pheromone(m,n) = 0;
			}
		}
		// Add some other condition for cutting off algorithm early if clear optimum is being reached
	}
}
