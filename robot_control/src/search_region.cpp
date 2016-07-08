#include <robot_control/search_region.h>

SearchRegion::SearchRegion()
{
	roiTimer = nh.createTimer(ros::Duration(480.0), &SearchRegion::roiTimeExpiredCallback_, this); // 480 sec == 8 min; implement smarter way to compute
	roiTimer.stop();
	roiTimeExpired = false;
}

bool SearchRegion::runProc()
{
	//ROS_INFO("searchRegion state = %i", state);
	ROS_INFO_THROTTLE(1,"roiTimer.isValid = %i",roiTimer.isValid());
	ROS_INFO_THROTTLE(1,"roiTimer.hasPending = %i",roiTimer.hasPending());
	switch(state)
	{
	case _init_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		examineCount = 0;
		confirmCollectFailedCount = 0;
		if(!roiKeyframed) // check if ROI is not yet keyframed
		{
			ROS_INFO("ROI not keyframed");
			searchMapSrv.request.createMap = true;
			searchMapSrv.request.roiIndex = currentROIIndex;
			searchMapSrv.request.deleteMap = false;
			if(searchMapClient.call(searchMapSrv)) ROS_DEBUG("searchMap service call successful");
			else ROS_ERROR("searchMap service call unsuccessful");
			roiKeyframed = true;
			// start timer based on allocated time
			roiTimeExpired = false;
			roiTimer.stop();
			roiTimer.setPeriod(ros::Duration(allocatedROITime));
			roiTimer.start();
		}
		if(roiTimeExpired)
		{
			roiTimer.stop();
			// Set ROI to searched
			modROISrv.request.setSearchedROI = true;
			modROISrv.request.searchedROIState = true;
			modROISrv.request.numSearchedROI = currentROIIndex;
			modROISrv.request.addNewROI = false;
			if(modROIClient.call(modROISrv)) ROS_DEBUG("modify ROI service call successful");
			else ROS_ERROR("modify ROI service call unsuccessful");
			inSearchableRegion = false;
			// Delete searchLocalMap
			searchMapSrv.request.createMap = false;
			searchMapSrv.request.deleteMap = true;
			if(searchMapClient.call(searchMapSrv)) ROS_DEBUG("searchMap service call successful");
			else ROS_ERROR("searchMap service call unsuccessful");
			roiKeyframed = false;
			ROS_WARN("roiTimeExpired........");
			state = _finish_;
		}
		else
		{
			numWaypointsToTravel = numRandomWaypoints; // Set number of waypoints to choose
			numWaypointsToPlan = numRandomWaypoints+1; // Number to plan path through is one more, because it includes the starting location
			chooseRandomWaypoints_(); // Select random waypoint locations based on sample prob distro on search local map
			// Add current location as last entry in waypointsToPlan
			currentLocation.x = robotStatus.xPos;
			currentLocation.y = robotStatus.yPos;
			waypointsToPlan.push_back(currentLocation);
			clearAndResizeWTT(); // waypointsToPlan minus the current location will get written into waypointsToTravel. Clear and resize to accomodate
			antColony_(); // Plan path through waypoints with ant colongy optimization algorithm
			// Do we really want to do this? Waypoints are very close together. callIntermediateWaypoints(); // Plan around obastacles.
			sendDriveAndSearch(252); // 252 = b11111100 -> cached = 1; purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
			state = _exec_;
		}
		break;
	case _exec_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		if(possibleSample || definiteSample && !(cvSamplesFoundMsg.procType==this->procType && cvSamplesFoundMsg.serialNum==this->serialNum)) // Found a possible or definite sample, but did not finish set of waypoints. Clear exec deque before moving on.
		{
			sendDequeClearAll();
			state = _finish_;
		}
		else if(cvSamplesFoundMsg.procType==this->procType && cvSamplesFoundMsg.serialNum==this->serialNum) state = _finish_; // Last search action ended, but nothing found. Finish the proc.
		else state = _exec_; // Not finished, keep executing.
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
		state = _init_;
		break;
	}
}

void SearchRegion::roiTimeExpiredCallback_(const ros::TimerEvent &event)
{
	roiTimeExpired = true;
	ROS_WARN("roiTimeExpiredCallback");
}

void SearchRegion::chooseRandomWaypoints_()
{
	randomSearchWaypointsSrv.request.numSeachWaypoints = numRandomWaypoints;
	if(randomSearchWaypointsClient.call(randomSearchWaypointsSrv))
	{
		ROS_DEBUG("randomSearchWaypoints service call successful");
		waypointsToPlan.clear();
		waypointsToPlan = randomSearchWaypointsSrv.response.waypointList;
	}
	else ROS_ERROR("randomSearchWaypoints service call unsuccessful");
}

void SearchRegion::antColony_()
{
	value.resize(numWaypointsToPlan);
	valueNormalized.resize(numWaypointsToPlan);
	pheromone.set_size(numWaypointsToPlan,numWaypointsToPlan);
	pheromone.fill(initialPheromone);
	ROS_INFO("pheromone matrix, beginning:");
	pheromone.print();
	//ROS_INFO("numWaypointsToPlan = %i",numWaypointsToPlan);
	distance.set_size(numWaypointsToPlan,numWaypointsToPlan);
	//ROS_INFO("distance rows = %i, distance cols = %i",distance.n_rows,distance.n_cols);
	//ROS_INFO("waypointsToPlan.size() = %i",waypointsToPlan.size());
	terrainHazard.set_size(numWaypointsToPlan,numWaypointsToPlan);
	for(int m=0; m<numWaypointsToPlan; m++)
	{
		for(int n=0; n<numWaypointsToPlan; n++)
		{
			//ROS_INFO("before distance matrix calc: m=%i n=%i",m,n);
			distance(m,n) = hypot(waypointsToPlan.at(m).x - waypointsToPlan.at(n).x,
								  waypointsToPlan.at(m).y - waypointsToPlan.at(n).y);
			//ROS_INFO("after distance matrix calc");
			terrainHazard.fill(0.0); // Temporary until actual terrain hazard calculation implemented
			// make service requests to get terrain hazard info
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
			valueSum = 0.0;
			valueNormalizedSum = 0.0;
			valueNormalizedFloor = 0.0;
			for(j; j<numWaypointsToPlan; j++)
			{
				/*ROS_DEBUG("before value computation, j=%i",j);
				ROS_DEBUG("pheromone(i,j) = %f",pheromone(i,j));
				ROS_DEBUG("waypointsToPlan.at(j).sampleProb = %f",waypointsToPlan.at(j).sampleProb);
				ROS_DEBUG("sampleProbGain*sampleProb = %f",sampleProbGain*waypointsToPlan.at(j).sampleProb);
				ROS_DEBUG("pheromoneGain*pheromone(i,j) = %f",pheromoneGain*pheromone(i,j));
				ROS_DEBUG("distance(i,j) = %f",distance(i,j));
				ROS_DEBUG("distanceGain*distance(i,j) = %f",distanceGain*distance(i,j));
				ROS_DEBUG("terrainHazard(i,j) = %f",terrainHazard(i,j));
				ROS_DEBUG("terrainGain*terrainHazard(i,j) = %f",terrainGain*terrainHazard(i,j));
				ROS_DEBUG("notVisited[j] = %i",notVisited.at(j));*/
				computedValue =	(sampleProbGain*waypointsToPlan.at(j).sampleProb +
								pheromoneGain*pheromone(i,j) -
								distanceGain*distance(i,j) -
								terrainGain*terrainHazard(i,j));
				//ROS_DEBUG("after value computation, before coersion. value[j] = %f",value.at(j));
				if(computedValue <= 0.0 && notVisited.at(j)==1) value.at(j) = 0.001;
				else value.at(j) = ((float)notVisited.at(j))*computedValue;
				//ROS_DEBUG("after value computation. value[j] = %f",value.at(j));
			}
			for(int k=0; k<numWaypointsToPlan; k++) valueSum += value.at(k);
			//ROS_DEBUG("valueSum = %f",valueSum);
			for(int k=0; k<numWaypointsToPlan; k++)
			{
				valueNormalized.at(k) = value.at(k)/valueSum;
				if(value.at(k)!=0.0 && valueNormalized.at(k)==0.0) valueNormalized.at(k) = 0.001; // Basement for normalized values. Probability distribution is discrete, not continuous, so there needs to be a basement (0.1%) that even the smallest probabilities get rounded up to so they are included in the distribution and not lost due to discrete rounding down
			}
			for(int k=0; k<numWaypointsToPlan; k++) valueNormalizedSum += valueNormalized.at(k);
			//ROS_DEBUG("valueNormalizedSum = %f",valueNormalizedSum);
			randomValue = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
			ROS_INFO("randomValue = %f",randomValue);
			if(randomValue>=valueNormalizedSum) // Picked 1.0 as the random value. Boundary condition, just choose the one with the biggest normalized value
			{
				//ROS_INFO("randomValue >= valueNormalizedSum");
				largestNormalizedValue = 0.0;
				for(int k=0; k<numWaypointsToPlan; k++)
				{
					if(valueNormalized.at(k)>largestNormalizedValue) {largestNormalizedValue = valueNormalized.at(k); bestJ = k;}
				}
			}
			else // Normal, non-boundary condition behavior where 0.0 <= randomValue < 1.0
			{
				//ROS_INFO("randomValue < valueNormalizedSum");
				for(int k=0; k<numWaypointsToPlan; k++)
				{
					/*ROS_DEBUG("k = %i",k);
					ROS_DEBUG("value[k] = %f",value.at(k));
					ROS_DEBUG("valueNormalized[k] = %f",valueNormalized.at(k));
					ROS_DEBUG("valueNormalizedFloor = %f",valueNormalizedFloor);
					ROS_DEBUG("randomValue = %f",randomValue);
					ROS_DEBUG("valueNormalizedCeiling = %f\n",valueNormalizedFloor + valueNormalized.at(k));*/
					if(randomValue >= valueNormalizedFloor && randomValue < (valueNormalizedFloor + valueNormalized.at(k)) && valueNormalized.at(k)!=0.0) {bestJ = k; break;}
					else valueNormalizedFloor += valueNormalized.at(k);
				}
			}
			//ROS_DEBUG("before pheromone increment, bestJ=%i, i=%i",bestJ,i);
			//ROS_DEBUG("pheroDepoGain/distance(i,bestJ) = %f\n", pheroDepoGain/distance(i,bestJ));
			pheromone(i,bestJ) += (pheroDepoGain/distance(i,bestJ) + pheroDecayValue);
			pheromone(bestJ,i) += (pheroDepoGain/distance(bestJ,i) + pheroDecayValue);
			//ROS_DEBUG("after pheromone increment");
			notVisited.at(bestJ) = 0;
			i = bestJ;
			j = 0;
			notVisitedSum = 0;
			for(int k=0; k<numWaypointsToPlan; k++) notVisitedSum += notVisited.at(k);
		}
		for(int m=0; m<numWaypointsToPlan; m++) // Pheromone decay
		{
			for(int n=0; n<numWaypointsToPlan; n++)
			{
				pheromone(m,n) -= pheroDecayValue;
				if(pheromone(m,n)<0.0) pheromone(m,n) = 0.0;
			}
		}
		//ROS_INFO("pheromone matrix, loop:");
		//pheromone.print();
		// Add some other condition for cutting off algorithm early if clear optimum is being reached
	}
	ROS_INFO("pheromone matrix, end:");
	pheromone.print();
	i = numWaypointsToPlan - 1;
	notVisited.clear();
	notVisited.resize(numWaypointsToPlan,1);
	notVisited.back() = 0; // Do not want to include current location in possible set of waypoints to visit
	notVisitedSum = numWaypointsToPlan;
	for(int o=0; o<numWaypointsToTravel; o++)
	{
		bestPheromone = 0.0;
		j=0;
		for(j; j<numWaypointsToPlan; j++)
		{
			/*ROS_DEBUG("i = %i", i);
			ROS_DEBUG("j = %i", j);
			ROS_DEBUG("notVisited.at(j) = %i", notVisited.at(j));
			ROS_DEBUG("pheromone(i,j) = %f", pheromone(i,j));
			ROS_DEBUG("bestPheromone = %f", bestPheromone);*/
			if((notVisited.at(j)*pheromone(i,j))>bestPheromone) {bestPheromone = pheromone(i,j); bestJ = j; ROS_INFO("bestPheromone found, bestJ = %i", bestJ);}
		}
		waypointsToTravel.at(o) = waypointsToPlan.at(bestJ);
		notVisited.at(bestJ) = 0;
		i = bestJ;
		ROS_INFO("waypointsToTravel[%i]: x = %f  y = %f", o, waypointsToTravel.at(o).x, waypointsToTravel.at(o).y);
	}
}
