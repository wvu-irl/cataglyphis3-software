#include <robot_control/mission_planning.h>

MissionPlanning::MissionPlanning()
{
	woiClient = nh.serviceClient<robot_control::WaypointsOfInterest>("/control/mapmanager/waypointsofinterest");
    execActionClient = nh.serviceClient<messages::ExecAction>("control/exec/actionin");
	navSub = nh.subscribe<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout", 1, &MissionPlanning::navCallback_, this);
    ExecActionEndedSub = nh.subscribe<messages::ExecActionEnded>("control/exec/dequeempty", 1, &MissionPlanning::ExecActionEndedCallback_, this);
    intermediateWaypointsClient = nh.serviceClient<robot_control::IntermediateWaypoints>("/control/safepathing/intermediatewaypoints");
    reqROIClient = nh.serviceClient<robot_control::RegionsOfInterest>("/control/mapmanager/regionsofinterest");
    modROIClient = nh.serviceClient<robot_control::ModifyROI>("/control/mapmanager/modifyroi");
    nb1Sub = nh.subscribe<messages::nb1_to_i7_msg>("hw_interface/nb1in/nb1in", 1, &MissionPlanning::nb1Callback_, this);
    collisionSub = nh.subscribe<messages::CollisionOut>("lidar/collisiondetectionout/collisiondetectionout", 1, &MissionPlanning::collisionCallback_, this);
    execInfoSub = nh.subscribe<messages::ExecInfo>("control/exec/info", 1, &MissionPlanning::execInfoCallback_, this);
	woiSrv.request.easyThresh = 0;
	woiSrv.request.medThresh = 0;
	woiSrv.request.hardThresh = 0;
    collisionInterruptTrigger = false;
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
    pauseStarted = false;
    robotStatus.pauseSwitch = true;
    execDequeEmpty = true;
    execLastProcType = deposit__;
    execLastSerialNum = 99;
    collisionInterruptThresh = 1.0; // m
    avoid.reg(avoid__);
    chooseRegion.reg(chooseRegion__); // Replace with loop over array of ptrs to objs. Also consider polymorphic constructor
    for(int i=0; i<NUM_PROC_TYPES; i++)
    {
        procsToExecute.push_back(false);
        procsToInterrupt.push_back(false);
        procsBeingExecuted.push_back(false);
    }
}

void MissionPlanning::run()
{
    //ROS_DEBUG("before evalConditions");
    evalConditions_();
    //ROS_DEBUG("after evalConditions");
    if(robotStatus.pauseSwitch) runPause_();
    else runProcesses_();
    
}

void MissionPlanning::avoidObstacle_()
{
    commandedAvoidObstacle = true;
    //sendDriveRel_(/*avoidMsg*/);
}

void MissionPlanning::returnHome_()
{
/*    commandedReturnHome = true;
    numWaypointsToTravel = 1;
    clearAndResizeWTT_();
    waypointsToTravel.at(0).x = homeX;
    waypointsToTravel.at(0).y = homeY;
    callIntermediateWaypoints_();
    sendDriveGlobal_();*/
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
/*    // *** Change this function to focus on path planning using ant colony within a region ***
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
    sendDriveGlobal_();*/
}

void MissionPlanning::chooseRegion_()
{
    
}

void MissionPlanning::init_()
{

}

void MissionPlanning::evalConditions_()
{
    for(int i; i<NUM_PROC_TYPES; i++) {procsToExecute.at(i) = false; procsToInterrupt.at(i) = false;}
    if(collisionMsg.collision!=0 || procsBeingExecuted.at(avoid__))
    {
        //if((collisionMsg.distance_to_collision <= collisionInterruptThresh) && procsBeingExecuted.at(avoid__)) procsToInterrupt.at(avoid__) = true;
        procsToExecute.at(avoid__) = true;
        if(procsBeingExecuted.at(chooseRegion__)) {procsToInterrupt.at(chooseRegion__) = true; ROS_INFO("to interrupt chooseRegion");}
        ROS_INFO("to execute avoid");
    }
    else
    {
        // something for commandedAvoidObstacle
        // something for commandedReturnHome
        // something for commandDeposit
        // something for commandedAcquire
        // something for commmandedExamine
        // something for commandedPlanRegionPath
        procsToExecute.at(chooseRegion__) = true;
        ROS_INFO("to execute chooseRegion");
        // something for commandedInit
    }
}

void MissionPlanning::runProcesses_()
{
    if(pauseStarted == true) pause.sendUnPause();
    pauseStarted = false;
    if(procsToExecute.at(chooseRegion__) || procsBeingExecuted.at(chooseRegion__)) chooseRegion.run(); // Change to array to make use of inherrited method
    if(procsToExecute.at(avoid__) || procsBeingExecuted.at(avoid__)) avoid.run();
}

void MissionPlanning::runPause_()
{
    if(pauseStarted == false) pause.sendPause();
    pauseStarted = true;
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
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!clearAndResizeWTT_(); !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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

void MissionPlanning::navCallback_(const messages::NavFilterOut::ConstPtr& msg)
{
	robotStatus.xPos = msg->x_position;
	robotStatus.yPos = msg->y_position;
	robotStatus.heading = msg->heading;
	robotStatus.bearing = msg->bearing;
}

void MissionPlanning::ExecActionEndedCallback_(const messages::ExecActionEnded::ConstPtr &msg)
{
    execDequeEmpty = msg->dequeEmpty;
    execLastProcType = static_cast<PROC_TYPES_T>(msg->procType);
    execLastSerialNum = msg->serialNum;
}

void MissionPlanning::nb1Callback_(const messages::nb1_to_i7_msg::ConstPtr& msg)
{
    if(msg->pause_switch==0) robotStatus.pauseSwitch = false;
    else robotStatus.pauseSwitch = true;
}

void MissionPlanning::collisionCallback_(const messages::CollisionOut::ConstPtr &msg)
{
    collisionMsg = *msg;
}

void MissionPlanning::execInfoCallback_(const messages::ExecInfo::ConstPtr &msg)
{
    execInfoMsg = *msg;
}
