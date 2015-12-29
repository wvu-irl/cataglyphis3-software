#include <robot_control/mission_planning.h>

MissionPlanning::MissionPlanning()
{
	woiClient = nh.serviceClient<robot_control::WaypointsOfInterest>("/control/mapmanager/waypointsofinterest");
	execActionPub = nh.advertise<messages::ExecAction>("control/exec/actionin", 1000);
	woiSrv.request.easyThresh = 0;
	woiSrv.request.medThresh = 0;
	woiSrv.request.hardThresh = 0;
}

void MissionPlanning::run()
{
	if(1/*something intelligent*/) planNewPath_();
}

void MissionPlanning::planNewPath_()
{
	// Decision on how to set thresholds, based on which type of samples are being searched for. Constant for now
	woiSrv.request.easyThresh = 200;
	woiSrv.request.medThresh = 200;
	woiSrv.request.hardThresh = 200;
	if(woiSrv.request.easyThresh > 1000) includeEasy = 0;
	else includeEasy = 1;
	if(woiSrv.request.medThresh > 1000) includeMed = 0;
	else includeMed = 1;
	if(woiSrv.request.hardThresh > 1000) includeHard = 0;
	else includeHard = 1;
	if(woiClient.call(woiSrv)) ROS_INFO("woi service call successful");
	else ROS_ERROR("woi service call unsuccessful");
	currentLocation.x = 0.0; // Replace with actual current x and y
	currentLocation.y = 0.0; // ***
	woiSrv.response.waypointArray.push_back(currentLocation);
	numInterestingWaypoints = woiSrv.response.waypointArray.size();
	antColony_();
	for(int i=0; i<numInterestingWaypoints; i++)
	{
		execActionMsg.newActionFlag = 1;
		execActionMsg.float1 = waypointsToTravel.at(i).x;
		execActionMsg.float2 = waypointsToTravel.at(i).y;
		execActionMsg.float3 = 1.5;
		execActionMsg.float4 = 45.0;
		execActionMsg.bool1 = false;
		execActionPub.publish(execActionMsg);
	}
}

void MissionPlanning::antColony_()
{
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
	i = numInterestingWaypoints - 1;
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
				value.at(j) = notVisited.at(j)*(includeEasy*easyProbGain*woiSrv.response.waypointArray.at(j).easyProb +
												includeMed*medProbGain*woiSrv.response.waypointArray.at(j).medProb +
												includeHard*hardProbGain*woiSrv.response.waypointArray.at(j).hardProb +
												pheromoneGain*pheromone(i,j) -
												distanceGain*distance(i,j) -
												terrainGain*terrainHazard(i,j)) /
												(includeEasy*easyProbGain + includeMed*medProbGain + includeHard*hardProbGain + pheromoneGain);
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
			notVisitedSum = 0;
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
	waypointsToTravel.resize(numInterestingWaypoints - 1);
	i = numInterestingWaypoints - 1;
	bestPheromone = 0;
	notVisited.clear();
	notVisited.resize(numInterestingWaypoints,1);
	notVisited.back() = 0; // Do not want to include current location in possible set of waypoints to visit
	notVisitedSum = numInterestingWaypoints;
	for(int o=0; o<numInterestingWaypoints; o++)
	{
		j=0;
		for(j; j<numInterestingWaypoints; j++)
		{
			if((notVisited.at(j)*pheromone(i,j))>bestPheromone) {bestPheromone = pheromone(i,j); bestJ = j;}
		}
		waypointsToTravel.at(o) = woiSrv.response.waypointArray.at(bestJ);
		notVisited.at(bestJ) = 0;
		i = bestJ;
	}
}
