#include <robot_control/search_region.h>

SearchRegion::SearchRegion()
{
	roiTimer = nh.createTimer(ros::Duration(allocatedROITime), &SearchRegion::roiTimeExpiredCallback_, this, true);
}

bool SearchRegion::runProc()
{
	ROS_INFO("searchRegion state = %i", state);
	switch(state)
	{
	case _init_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		if(!roiKeyframed) // check if ROI is not yet keyframed
		{
			searchMapSrv.request.createMap = true;
			searchMapSrv.request.roiIndex = currentROIIndex;
			searchMapSrv.request.deleteMap = false;
			if(searchMapClient.call(searchMapSrv)) ROS_DEBUG("searchMap service call successful");
			else ROS_ERROR("searchMap service call unsuccessful");
			// start timer based on allocated time
			roiTimeExpired = false;
			roiTimer.stop();
			roiTimer.setPeriod(ros::Duration(allocatedROITime));
			roiTimer.start();
		}
		if(roiTimeExpired)
		{
			modROISrv.request.setSearchedROI = true;
			modROISrv.request.numSearchedROI = currentROIIndex;
			modROISrv.request.searchedROIState = true;
			inSearchableRegion = false;
			searchMapSrv.request.createMap = false;
			searchMapSrv.request.roiIndex = 0;
			searchMapSrv.request.deleteMap = true;
			if(searchMapClient.call(searchMapSrv)) ROS_DEBUG("searchMap service call successful");
			else ROS_ERROR("searchMap service call unsuccessful");
			state = _finish_;
		}
		else
		{
			numWaypointsToTravel = numRandomWaypoints; // Set number of waypoints to choose
			numWaypointsToPlan = numRandomWaypoints+1; // Number to plan path through is one more, because it includes the starting location
			chooseRandomWaypoints_(); // Select random waypoint locations based on sample prob distro on search local map
			// Add current location as last entry in waypointsToPlan
			clearAndResizeWTT(); // waypointsToPlan minus the current location will get written into waypointsToTravel. Clear and resize to accomodate
			antColony_(); // Plan path through waypoints with ant colongy optimization algorithm
			// Do we really want to do this? Waypoints are very close together. callIntermediateWaypoints(); // Plan around obastacles.
			sendDriveAndSearch(124); // 124 = b1111100 -> purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
			state = _exec_;
		}
		break;
	case _exec_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		if(cvSamplesFoundMsg.procType==this->procType && cvSamplesFoundMsg.serialNum==this->serialNum) state = _finish_;
		else state = _exec_;
		break;
	case _interrupt_:
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _exec_;
		break;
	case _finish_:
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		break;
	}
}

void SearchRegion::roiTimeExpiredCallback_()
{
	roiTimeExpired = true;
}

void SearchRegion::chooseRandomWaypoints_()
{

}

void SearchRegion::antColony_()
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
