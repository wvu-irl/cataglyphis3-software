#include <robot_control/mission_planning.h>

MissionPlanning::MissionPlanning()
{
	woiClient = nh.serviceClient<robot_control::WaypointsOfInterest>("/control/mapmanager/waypointsofinterest");
	execActionPub = nh.advertise<messages::ExecAction>("control/exec/actionin", 1000);
	navSub = nh.subscribe<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout", 1, &MissionPlanning::navCallback_, this);
	woiSrv.request.easyThresh = 0;
	woiSrv.request.medThresh = 0;
	woiSrv.request.hardThresh = 0;
}

void MissionPlanning::run()
{
	if(1/*something intelligent*/) planRegionPath_();
}

void MissionPlanning::planRegionPath_()
{
	// Decision on how to set thresholds, based on which type of samples are being searched for. Constant for now
	woiSrv.request.easyThresh = 200;
	woiSrv.request.medThresh = 200;
	woiSrv.request.hardThresh = 200;
	// ********************************
	if(woiSrv.request.easyThresh > 1000) includeEasy = 0;
	else includeEasy = 1;
	if(woiSrv.request.medThresh > 1000) includeMed = 0;
	else includeMed = 1;
	if(woiSrv.request.hardThresh > 1000) includeHard = 0;
	else includeHard = 1;
	if(woiClient.call(woiSrv)) ROS_DEBUG("woi service call successful");
	else ROS_ERROR("woi service call unsuccessful");
	numWaypointsToPlan = woiSrv.response.waypointArray.size();
	waypointsToPlan.resize(numWaypointsToPlan);
	ROS_DEBUG("before waypointArray copy");
	std::copy(woiSrv.response.waypointArray.begin(), woiSrv.response.waypointArray.end(), waypointsToPlan.begin());
	ROS_DEBUG("after waypointArray copy");
	currentLocation.x = 0.0; // Replace with actual current x and y
	currentLocation.y = 0.0; // ***
	waypointsToPlan.push_back(currentLocation);
	ROS_DEBUG("before antColony_()");
	antColony_();
	ROS_DEBUG("after antColony_()");
	for(int i=0; i<numWaypointsToPlan; i++)
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
	value.resize(numWaypointsToPlan);
	valueNormalized.resize(numWaypointsToPlan);
	pheromone.set_size(numWaypointsToPlan,numWaypointsToPlan);
	pheromone.fill(initialPheromone);
	distance.set_size(numWaypointsToPlan,numWaypointsToPlan);
	terrainHazard.set_size(numWaypointsToPlan,numWaypointsToPlan);
	for(int m=0; m<numWaypointsToPlan; m++)
	{
		for(int n=0; n<numWaypointsToPlan; n++)
		{
			ROS_DEBUG("before distance matrix calc: m=%i n=%i",m,n);
			distance(m,n) = hypot(waypointsToPlan.at(m).x - waypointsToPlan.at(n).x,
								  waypointsToPlan.at(m).y - waypointsToPlan.at(n).y);
			ROS_DEBUG("after distance matrix calc");
			terrainHazard.fill(0); // Temporary until actual terrain hazard calculation implemented
			// terrainHazard(m,n) = something;
		}
	}
	antNum = 0;
	i = numWaypointsToPlan - 1;
	j = 0;
	for(antNum; antNum<maxAntNum; antNum++)
	{
		ROS_DEBUG("antNum = %i",antNum);
		notVisited.clear();
		notVisited.resize(numWaypointsToPlan,1);
		notVisited.back() = 0; // Do not want to include current location in possible set of waypoints to visit
		notVisitedSum = numWaypointsToPlan;
		while(notVisitedSum!=0)
		{
			valueSum = 0;
			valueNormalizedFloor = 0;
			for(j; j<numWaypointsToPlan; j++)
			{
				ROS_DEBUG("before value computation, j=%i",j);
				ROS_DEBUG("waypointsToPlan.at(j).easyProb = %i",waypointsToPlan.at(j).easyProb);
				ROS_DEBUG("waypointsToPlan.at(j).medProb = %i",waypointsToPlan.at(j).medProb);
				ROS_DEBUG("waypointsToPlan.at(j).hardProb = %i",waypointsToPlan.at(j).hardProb);
				ROS_DEBUG("pheromone(i,j) = %i",pheromone(i,j));
				ROS_DEBUG("distance(i,j) = %i",distance(i,j));
				ROS_DEBUG("terrainHazard(i,j) = %i",terrainHazard(i,j));
				value.at(j) = notVisited.at(j)*(includeEasy*easyProbGain*waypointsToPlan.at(j).easyProb +
												includeMed*medProbGain*waypointsToPlan.at(j).medProb +
												includeHard*hardProbGain*waypointsToPlan.at(j).hardProb +
												pheromoneGain*pheromone(i,j) -
												distanceGain*distance(i,j) -
												terrainGain*terrainHazard(i,j)) /
												(includeEasy*easyProbGain + includeMed*medProbGain + includeHard*hardProbGain + pheromoneGain);
				ROS_DEBUG("after value computation. value[j] = %i",value.at(j));
			}
			for(int k=0; k<numWaypointsToPlan; k++) valueSum += value.at(k);
			ROS_DEBUG("valueSum = %i",valueSum);
			for(int k=0; k<numWaypointsToPlan; k++) valueNormalized.at(k) = (1000*value.at(k))/valueSum;
			randomValue = rand() % 1001;
			for(int k=0; k<numWaypointsToPlan; k++)
			{
				ROS_DEBUG("k = %i",k);
				ROS_DEBUG("value[k] = %i",value.at(k));
				ROS_DEBUG("valueNormalized[k] = %i",valueNormalized.at(k));
				ROS_DEBUG("valueNormalizedFloor = %i",valueNormalizedFloor);
				ROS_DEBUG("randomValue = %i",randomValue);
				ROS_DEBUG("valueNormalizedCeiling = %i\n",valueNormalizedFloor + valueNormalized.at(k));
				if(randomValue >= valueNormalizedFloor && randomValue <= (valueNormalizedFloor + valueNormalized.at(k))) {bestJ = k; break;}
				else valueNormalizedFloor += valueNormalized.at(k);
			}
			ROS_DEBUG("before pheromone increment, bestJ=%i, i=%i",bestJ,i);
			pheromone(i,bestJ) += pheroDepoGain/distance(i,bestJ);
			pheromone(bestJ,i) += pheroDepoGain/distance(bestJ,i);
			ROS_DEBUG("after pheromone increment");
			notVisited.at(bestJ) = 0;
			i = bestJ;
			j = 0;
			notVisitedSum = 0;
			for(int k=0; k<numWaypointsToPlan; k++) notVisitedSum += notVisited.at(k);
		}
		for(int m=0; m<numWaypointsToPlan; m++)
		{
			for(int n=0; n<numWaypointsToPlan; n++)
			{
				pheromone(m,n) -= pheroDecayValue;
				if(pheromone(m,n)<0) pheromone(m,n) = 0;
			}
		}
		// Add some other condition for cutting off algorithm early if clear optimum is being reached
	}
	waypointsToTravel.resize(numWaypointsToPlan - 1);
	i = numWaypointsToPlan - 1;
	bestPheromone = 0;
	notVisited.clear();
	notVisited.resize(numWaypointsToPlan,1);
	notVisited.back() = 0; // Do not want to include current location in possible set of waypoints to visit
	notVisitedSum = numWaypointsToPlan;
	for(int o=0; o<numWaypointsToPlan; o++)
	{
		j=0;
		for(j; j<numWaypointsToPlan; j++)
		{
			if((notVisited.at(j)*pheromone(i,j))>bestPheromone) {bestPheromone = pheromone(i,j); bestJ = j;}
		}
		waypointsToTravel.at(o) = waypointsToPlan.at(bestJ);
		notVisited.at(bestJ) = 0;
		i = bestJ;
		ROS_DEBUG("waypointsToTravel[%i]: x = %f  y = %f", o, waypointsToTravel.at(o).x, waypointsToTravel.at(o).y);
	}
}

void MissionPlanning::navCallback_(const messages::NavFilterOut::ConstPtr& msg)
{
	robotStatus.xPos = msg->x_position;
	robotStatus.yPos = msg->y_position;
	robotStatus.heading = msg->heading;
	robotStatus.bearing = msg->bearing;
}
