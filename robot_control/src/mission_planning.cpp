#include <robot_control/mission_planning.h>

MissionPlanning::MissionPlanning()
{
	woiClient = nh.serviceClient<robot_control::WaypointsOfInterest>("/control/mapmanager/waypointsofinterest");
    execActionClient = nh.serviceClient<messages::ExecAction>("control/exec/actionin");
	navSub = nh.subscribe<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout", 1, &MissionPlanning::navCallback_, this);
    intermediateWaypointsClient = nh.serviceClient<robot_control::IntermediateWaypoints>("/control/safepathing/intermediatewaypoints");
	woiSrv.request.easyThresh = 0;
	woiSrv.request.medThresh = 0;
	woiSrv.request.hardThresh = 0;
    collisionDetected = false;
    commandedAvoidObstacle = false;
    possessingSample = false;
    commandedReturnHome = false;
    confirmedAcquire = false;
    commandedAcquire = false;
    interestingSampleNearby = false;
    commandedExamine = false;
    inIncompleteROI = false;
    commandedPlanRegionPath = false;
    completedROI = false;
    completedDeposit = false;
    commandedChooseRegion = false;
    initComplete = false;
    commandedInit = false;
}

void MissionPlanning::run()
{
    /*if(collisionDetected)
    {
        if(!commandedAvoidObstacle) {avoidObstacle_(); return;}
    }

    if(possessingSample)
    {
        if(!commandedReturnHome) {returnHome_(); deposit_(); return;}
    }

    if(confirmedAcquire)
    {
        if(!commandedAcquire) {acquire_(); return;}
    }

    if(interestingSampleNearby)
    {
        if(!commandedExamine) {examine_(); return;}
    }

    if(inIncompleteROI)
    {
        if(!commandedPlanRegionPath) {planRegionPath_(); return;}
    }*/

    if(completedROI || completedDeposit)
    {
        if(!commandedChooseRegion) {chooseRegion_(); return;}
    }

    /*if(!initComplete)
    {
        if(!commandedInit) {init_(); return;}
    }*/

    return;
}

void MissionPlanning::avoidObstacle_()
{
    commandedAvoidObstacle = true;
    //sendDriveRel_(/*avoidMsg*/);
}

void MissionPlanning::returnHome_()
{
    commandedReturnHome = true;
    numWaypointsToTravel = 1;
    clearAndResizeWTT_();
    waypointsToTravel.at(0).x = homeX;
    waypointsToTravel.at(0).y = homeY;
    callIntermediateWaypoints_();
    sendDriveGlobal_();
}

void MissionPlanning::deposit_()
{

}

void MissionPlanning::acquire_()
{

}

void MissionPlanning::examine_()
{

}

void MissionPlanning::planRegionPath_()
{
    // *** Change this function to focus on path planning using ant colony within a region ***
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
	numWaypointsToPlan = woiSrv.response.waypointArray.size() + 1;
	numWaypointsToTravel = numWaypointsToPlan - 1;
	waypointsToPlan.resize(numWaypointsToTravel);
	ROS_DEBUG("before waypointArray copy");
	std::copy(woiSrv.response.waypointArray.begin(), woiSrv.response.waypointArray.end(), waypointsToPlan.begin());
	ROS_DEBUG("after waypointArray copy");
	currentLocation.x = 0.0; // Replace with actual current x and y
	currentLocation.y = 0.0; // ***
	waypointsToPlan.push_back(currentLocation);
	ROS_DEBUG("before antColony_()");
	antColony_();
	ROS_DEBUG("after antColony_()");
    callIntermediateWaypoints_();
    sendDriveGlobal_();
}

void MissionPlanning::chooseRegion_()
{
    commandedChooseRegion = true;
    // Request info about regions
    if(reqROIClient.call(regionsOfInterestSrv)) ROS_DEBUG("regionsOfInterest service call successful");
    else ROS_ERROR("regionsOfInterest service call successful");

    // Loop through list and choose best region not yet visited
    bestROIValue = 0;
    bestROINum = 0;
    for(int i=0; i < regionsOfInterestSrv.response.waypointArray.size(); i++)
    {
        roiValue = (easyProbGain*regionsOfInterestSrv.response.waypointArray.at(i).easyProb +
                    medProbGain*regionsOfInterestSrv.response.waypointArray.at(i).medProb +
                    hardProbGain*regionsOfInterestSrv.response.waypointArray.at(i).hardProb +
                    distanceGain*hypot(regionsOfInterestSrv.response.waypointArray.at(i).x - robotStatus.xPos,
                                       regionsOfInterestSrv.response.waypointArray.at(i).y - robotStatus.yPos)/* -
                    terrainGain*terrainHazard(i,j)*/) /
                    (easyProbGain + medProbGain + hardProbGain);
        if(roiValue > bestROIValue) {bestROINum = i; bestROIValue = roiValue;}
    }

    // Call service to drive to center of the chosen region
    numWaypointsToTravel = 1;
    clearAndResizeWTT_();
    waypointsToTravel.at(0) = regionsOfInterestSrv.response.waypointArray.at(bestROINum);
    sendDriveGlobal_();
}

void MissionPlanning::init_()
{

}

void MissionPlanning::evalCommandedFlags_()
{
    // something for commandedAvoidObstacle
    // something for commandedReturnHome
    // something for commandedAcquire
    // something for commmandedExamine
    // something for commandedPlanRegionPath
    // something for commandDeposit

    // something for commandedInit
}

void MissionPlanning::sendDriveGlobal_()
{
    for(int i=0; i<numWaypointsToTravel; i++)
    {
        execActionSrv.request.nextActionType = _driveGlobal;
        execActionSrv.request.newActionFlag = 1;
        execActionSrv.request.pushToFrontFlag = false;
        execActionSrv.request.clearDequeFlag = false;
        execActionSrv.request.pause = false;
        execActionSrv.request.float1 = waypointsToTravel.at(i).x;
        execActionSrv.request.float2 = waypointsToTravel.at(i).y;
        execActionSrv.request.float3 = 1.5;
        execActionSrv.request.float4 = 45.0;
        execActionSrv.request.float5 = 0.0;
        execActionSrv.request.int1 = 0;
        execActionSrv.request.bool1 = false;
        if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
        else ROS_ERROR("exec action service call unsuccessful");
    }
}

void MissionPlanning::sendDriveRel_(float deltaDistance, float deltaHeading, bool endHeadingFlag, float endHeading, bool frontOfDeque)
{
    execActionSrv.request.nextActionType = _driveRelative;
    execActionSrv.request.newActionFlag = 1;
    execActionSrv.request.pushToFrontFlag = frontOfDeque;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.pause = false;
    execActionSrv.request.float1 = deltaDistance;
    execActionSrv.request.float2 = deltaHeading;
    execActionSrv.request.float3 = 1.5;
    execActionSrv.request.float4 = 45.0;
    execActionSrv.request.float5 = endHeading;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = endHeadingFlag;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void MissionPlanning::antColony_()
{
	value.resize(numWaypointsToPlan);
	valueNormalized.resize(numWaypointsToPlan);
	pheromone.set_size(numWaypointsToPlan,numWaypointsToPlan);
	pheromone.fill(initialPheromone);
	ROS_INFO("pheromone matrix, beginning:");
	pheromone.print();
	distance.set_size(numWaypointsToPlan,numWaypointsToPlan);
	terrainHazard.set_size(numWaypointsToPlan,numWaypointsToPlan);
	for(int m=0; m<numWaypointsToPlan; m++)
	{
		for(int n=0; n<numWaypointsToPlan; n++)
		{
			//ROS_DEBUG("before distance matrix calc: m=%i n=%i",m,n);
			distance(m,n) = hypot(waypointsToPlan.at(m).x - waypointsToPlan.at(n).x,
								  waypointsToPlan.at(m).y - waypointsToPlan.at(n).y);
			//ROS_DEBUG("after distance matrix calc");
			terrainHazard.fill(0); // Temporary until actual terrain hazard calculation implemented
			// terrainHazard(m,n) = something;
		}
	}
	antNum = 0;
	for(antNum; antNum<maxAntNum; antNum++)
	{
		//ROS_INFO("antNum = %i",antNum);
		i = numWaypointsToPlan - 1;
		j = 0;
		notVisited.clear();
		notVisited.resize(numWaypointsToPlan,1);
		notVisited.back() = 0; // Do not want to include current location in possible set of waypoints to visit
		notVisitedSum = numWaypointsToPlan;
		while(notVisitedSum!=0)
		{
			valueSum = 0;
			valueNormalizedSum = 0;
			valueNormalizedFloor = 0;
			for(j; j<numWaypointsToPlan; j++)
			{
				/*ROS_DEBUG("before value computation, j=%i",j);
				ROS_DEBUG("pheromone(i,j) = %i",pheromone(i,j));
				ROS_DEBUG("waypointsToPlan.at(j).easyProb = %i",waypointsToPlan.at(j).easyProb);
				ROS_DEBUG("easyProbGain*easyProb = %i",includeEasy*easyProbGain*waypointsToPlan.at(j).easyProb);
				ROS_DEBUG("waypointsToPlan.at(j).medProb = %i",waypointsToPlan.at(j).medProb);
				ROS_DEBUG("medProbGain*medProb = %i",includeMed*medProbGain*waypointsToPlan.at(j).medProb);
				ROS_DEBUG("waypointsToPlan.at(j).hardProb = %i",waypointsToPlan.at(j).hardProb);
				ROS_DEBUG("hardProbGain*hardProb = %i",includeHard*hardProbGain*waypointsToPlan.at(j).hardProb);
				ROS_DEBUG("pheromoneGain*pheromone(i,j) = %i",pheromoneGain*pheromone(i,j));
				ROS_DEBUG("distance(i,j) = %i",distance(i,j));
				ROS_DEBUG("distanceGain*distance(i,j) = %i",distanceGain*distance(i,j));
				ROS_DEBUG("terrainHazard(i,j) = %i",terrainHazard(i,j));
				ROS_DEBUG("terrainGain*terrainHazard(i,j) = %i",terrainGain*terrainHazard(i,j));
				ROS_DEBUG("notVisited[j] = %i",notVisited.at(j));*/
				computedValue =	(includeEasy*easyProbGain*waypointsToPlan.at(j).easyProb +
								includeMed*medProbGain*waypointsToPlan.at(j).medProb +
								includeHard*hardProbGain*waypointsToPlan.at(j).hardProb +
								pheromoneGain*pheromone(i,j) -
								distanceGain*distance(i,j) -
								terrainGain*terrainHazard(i,j)) /
								(includeEasy*easyProbGain + includeMed*medProbGain + includeHard*hardProbGain + pheromoneGain);
				//ROS_DEBUG("after value computation, before coersion. value[j] = %i",value.at(j));
				if(computedValue <= 0 && notVisited.at(j)==1) value.at(j) = 1;
				else value.at(j) = notVisited.at(j)*computedValue;
				//ROS_DEBUG("after value computation. value[j] = %i",value.at(j));
			}
			for(int k=0; k<numWaypointsToPlan; k++) valueSum += value.at(k);
			//ROS_DEBUG("valueSum = %i",valueSum);
			for(int k=0; k<numWaypointsToPlan; k++)
			{
				valueNormalized.at(k) = (1000*value.at(k))/valueSum;
				if(value.at(k)!=0 && valueNormalized.at(k)==0) valueNormalized.at(k) = 1; // Basement for normalized values. Probability distribution is discrete, not continuous, so there needs to be a basement (0.1%) that even the smallest probabilities get rounded up to so they are included in the distribution and not lost due to discrete rounding down
			}
			for(int k=0; k<numWaypointsToPlan; k++) valueNormalizedSum += valueNormalized.at(k);
			//ROS_DEBUG("valueNormalizedSum = %i",valueNormalizedSum);
			randomValue = rand() % 1000;
			if(randomValue>=valueNormalizedSum)
			{
				largestNormalizedValue = 0;
				for(int k=0; k<numWaypointsToPlan; k++)
				{
					if(valueNormalized.at(k)>largestNormalizedValue) {largestNormalizedValue = valueNormalized.at(k); bestJ = k;}
				}
			}
			else
			{
				for(int k=0; k<numWaypointsToPlan; k++)
				{
					/*ROS_DEBUG("k = %i",k);
					ROS_DEBUG("value[k] = %i",value.at(k));
					ROS_DEBUG("valueNormalized[k] = %i",valueNormalized.at(k));
					ROS_DEBUG("valueNormalizedFloor = %i",valueNormalizedFloor);
					ROS_DEBUG("randomValue = %i",randomValue);
					ROS_DEBUG("valueNormalizedCeiling = %i\n",valueNormalizedFloor + valueNormalized.at(k));*/
					if(randomValue >= valueNormalizedFloor && randomValue < (valueNormalizedFloor + valueNormalized.at(k)) && valueNormalized.at(k)!=0) {bestJ = k; break;}
					else valueNormalizedFloor += valueNormalized.at(k);
				}
			}
			//ROS_DEBUG("before pheromone increment, bestJ=%i, i=%i",bestJ,i);
			//ROS_DEBUG("pheroDepoGain/distance(i,bestJ) = %i\n", pheroDepoGain/distance(i,bestJ));
			pheromone(i,bestJ) += (pheroDepoGain/distance(i,bestJ) + pheroDecayValue);
			pheromone(bestJ,i) += (pheroDepoGain/distance(bestJ,i) + pheroDecayValue);
			//ROS_DEBUG("after pheromone increment");
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
		//ROS_INFO("pheromone matrix, loop:");
		//pheromone.print();
		// Add some other condition for cutting off algorithm early if clear optimum is being reached
	}
	ROS_INFO("pheromone matrix, end:");
	pheromone.print();
    clearAndResizeWTT_();
	i = numWaypointsToPlan - 1;
	notVisited.clear();
	notVisited.resize(numWaypointsToPlan,1);
	notVisited.back() = 0; // Do not want to include current location in possible set of waypoints to visit
	notVisitedSum = numWaypointsToPlan;
	for(int o=0; o<numWaypointsToTravel; o++)
	{
		bestPheromone = 0;
		j=0;
		for(j; j<numWaypointsToPlan; j++)
		{
			/*ROS_DEBUG("i = %i", i);
			ROS_DEBUG("j = %i", j);
			ROS_DEBUG("notVisited.at(j) = %i", notVisited.at(j));
			ROS_DEBUG("pheromone(i,j) = %i", pheromone(i,j));
			ROS_DEBUG("bestPheromone = %i", bestPheromone);*/
			if((notVisited.at(j)*pheromone(i,j))>bestPheromone) {bestPheromone = pheromone(i,j); bestJ = j; ROS_DEBUG("bestPheromone found, bestJ = %i", bestJ);}
		}
		waypointsToTravel.at(o) = waypointsToPlan.at(bestJ);
		notVisited.at(bestJ) = 0;
		i = bestJ;
		ROS_DEBUG("waypointsToTravel[%i]: x = %f  y = %f", o, waypointsToTravel.at(o).x, waypointsToTravel.at(o).y);
	}
}

inline void MissionPlanning::clearAndResizeWTT_()
{
    waypointsToTravel.clear();
    waypointsToTravel.resize(numWaypointsToTravel);
}

void MissionPlanning::callIntermediateWaypoints_()
{
    initNumWaypointsToTravel = numWaypointsToTravel;
    totalIntermWaypoints = 0;
    for(int i = 0; i < initNumWaypointsToTravel; i++)
    {
        intermWaypointsIt = waypointsToTravel.begin() + i + totalIntermWaypoints;
        if(i == 0)
        {
            intermediateWaypointsSrv.request.start.x = robotStatus.xPos;
            intermediateWaypointsSrv.request.start.y = robotStatus.yPos;
        }
        else
        {
            intermediateWaypointsSrv.request.start.x = (*(intermWaypointsIt - 1)).x;
            intermediateWaypointsSrv.request.start.y = (*(intermWaypointsIt - 1)).y;
        }
        intermediateWaypointsSrv.request.finish.x = (*intermWaypointsIt).x;
        intermediateWaypointsSrv.request.finish.y = (*intermWaypointsIt).y;
        if(intermediateWaypointsClient.call(intermediateWaypointsSrv)) ROS_DEBUG("intermediateWaypoints service call successful");
        else ROS_ERROR("intermediateWaypoints service call unsuccessful");
        if(intermediateWaypointsSrv.response.waypointArray.size() > 0)
        {
            waypointsToTravel.insert(intermWaypointsIt,intermediateWaypointsSrv.response.waypointArray.begin(),intermediateWaypointsSrv.response.waypointArray.end());
            totalIntermWaypoints += intermediateWaypointsSrv.response.waypointArray.size();
            numWaypointsToTravel += intermediateWaypointsSrv.response.waypointArray.size();
        }
    }
}

void MissionPlanning::navCallback_(const messages::NavFilterOut::ConstPtr& msg)
{
	robotStatus.xPos = msg->x_position;
	robotStatus.yPos = msg->y_position;
	robotStatus.heading = msg->heading;
	robotStatus.bearing = msg->bearing;
}
