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

#include <robot_control/search_region.h>

bool SearchRegion::runProc()
{
    //ROS_INFO("searchRegion state = %i", state);
	//ROS_INFO_THROTTLE(1, "executing searchRegion");
	switch(state)
	{
	case _init_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        clearSampleHistory();
		avoidCount = 0;
		prevAvoidCountDecXPos = robotStatus.xPos;
		prevAvoidCountDecYPos = robotStatus.yPos;
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
            if(!timers[_roiTimer_]->running) {timers[_roiTimer_]->setPeriod(allocatedROITime); timers[_roiTimer_]->start();}
		}
		if(roiTimeExpired)
		{
			roiTimeExpired = false;
			timers[_roiTimer_]->stop();
			// Set ROI to searched
            modROISrv.request.setHardLockoutROI = false;
            modROISrv.request.hardLockoutROIState = false;
            modROISrv.request.modROIIndex = currentROIIndex;
            modROISrv.request.setSampleProps = true;
            if(currentROIIndex == 0) modROISrv.request.sampleProb = 0.0;
            else modROISrv.request.sampleProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).sampleProb*roiTimeExpiredNewSampleProbMultiplier;
            modROISrv.request.sampleSig = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).sampleSig;
            modROISrv.request.whiteProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).whiteProb;
            modROISrv.request.silverProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).silverProb;
            modROISrv.request.blueOrPurpleProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).blueOrPurpleProb;
            modROISrv.request.pinkProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).pinkProb;
            modROISrv.request.redProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).redProb;
            modROISrv.request.orangeProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).orangeProb;
            modROISrv.request.yellowProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).yellowProb;
			modROISrv.request.addNewROI = false;
            modROISrv.request.editGroup = false;
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
			//ROS_INFO("current ROI index = %i",currentROIIndex);
			ROS_INFO("choose random waypoints");
			chooseRandomWaypoints_(); // Select random waypoint locations based on sample prob distro on search local map
			numWaypointsToTravel = waypointsToPlan.size(); // Set number of waypoints to choose
			//ROS_INFO("numWaypointsToTravel = %i", numWaypointsToTravel);
			//for(int i=0; i<numWaypointsToTravel; i++) ROS_INFO("waypointsToPlan(%i) = (%f,%f)", i, waypointsToPlan.at(i).x, waypointsToPlan.at(i).y);
			numWaypointsToPlan = numWaypointsToTravel+1; // Number to plan path through is one more, because it includes the starting location
			// Add current location as last entry in waypointsToPlan
			currentLocation.x = robotStatus.xPos;
			currentLocation.y = robotStatus.yPos;
			waypointsToPlan.push_back(currentLocation);
			clearAndResizeWTT(); // waypointsToPlan minus the current location will get written into waypointsToTravel. Clear and resize to accomodate
			ROS_INFO("ant colony");
			antColony_(); // Plan path through waypoints with ant colongy optimization algorithm
			// Do we really want to do this? Waypoints are very close together. callIntermediateWaypoints(); // Plan around obastacles.
            sendDriveAndSearch();
			state = _exec_;
		}
		computeDriveSpeeds();
        resetQueueEmptyCondition();
		break;
	case _exec_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		computeDriveSpeeds();
		serviceAvoidCounterDecrement();
        if((possibleSample || definiteSample) && !searchEnded()) // Found a possible or definite sample, but did not finish set of waypoints. Clear exec deque before moving on.
		{
			sendDequeClearAll();
			state = _finish_;
		}
        else if(searchEnded() || queueEmptyTimedOut) state = _finish_; // Last search action ended, but nothing found. Finish the proc.
		else state = _exec_; // Not finished, keep executing.
        serviceQueueEmptyCondition();
		break;
	case _interrupt_:
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _exec_;
		break;
	case _finish_:
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		state = _init_;
		break;
	}
}

void SearchRegion::chooseRandomWaypoints_()
{
	randomSearchWaypointsSrv.request.numSearchWaypoints = numRandomWaypoints;
	if(randomSearchWaypointsClient.call(randomSearchWaypointsSrv))
	{
		ROS_DEBUG("randomSearchWaypoints service call successful");
		waypointsToPlan.clear();
		waypointsToPlan = randomSearchWaypointsSrv.response.waypointList;
		ROS_INFO("waypointsToPlan = %u", waypointsToPlan.size());
	}
	else ROS_ERROR("randomSearchWaypoints service call unsuccessful");
}

void SearchRegion::antColony_()
{
	value.resize(numWaypointsToPlan);
	valueNormalized.resize(numWaypointsToPlan);
	pheromone.set_size(numWaypointsToPlan,numWaypointsToPlan);
	pheromone.fill(initialPheromone);
	//ROS_INFO("pheromone matrix, beginning:");
	//pheromone.print();
	//ROS_INFO("numWaypointsToPlan = %i",numWaypointsToPlan);
	distance.set_size(numWaypointsToPlan,numWaypointsToPlan);
	//ROS_INFO("distance rows = %i, distance cols = %i",distance.n_rows,distance.n_cols);
	//ROS_INFO("waypointsToPlan.size() = %i",waypointsToPlan.size());
	terrainHazard.set_size(numWaypointsToPlan,numWaypointsToPlan);
	//ROS_INFO("after mat inits");
	for(int m=0; m<numWaypointsToPlan; m++)
	{
		for(int n=0; n<numWaypointsToPlan; n++)
		{
			//ROS_INFO("before distance matrix calc: m=%i n=%i",m,n);
			distance(m,n) = hypot(waypointsToPlan.at(m).x - waypointsToPlan.at(n).x,
								  waypointsToPlan.at(m).y - waypointsToPlan.at(n).y);
            if(m!=n && distance(m,n)==0.0) {distance(m,n) = 0.1; ROS_WARN("ant colony calculate distance == 0.0");} // Guard in case of incidental error to prevent divide by zero
			//ROS_INFO("after distance matrix calc");
			//terrainHazard.fill(0.0);
			// make service requests to get terrain hazard info
			if(distance(m,n) != 0.0)
			{
				searchLocalMapPathHazardsSrv.request.xStart = waypointsToPlan.at(m).x;
				searchLocalMapPathHazardsSrv.request.yStart = waypointsToPlan.at(m).y;
				searchLocalMapPathHazardsSrv.request.xEnd = waypointsToPlan.at(n).x;
				searchLocalMapPathHazardsSrv.request.yEnd = waypointsToPlan.at(n).y;
				searchLocalMapPathHazardsSrv.request.width = hazardCorridorWidth;
				if(searchLocalMapPathHazardsClient.call(searchLocalMapPathHazardsSrv)) ROS_DEBUG("searchLocalMapPathHazards service call successful");
				else ROS_ERROR("searchLocalMapPathHazards service call unsuccessful");
				terrainHazard(m,n) = searchLocalMapPathHazardsSrv.response.hazardValue;
			}
			else terrainHazard(m,n) = 0.0;
		}
	}
    /*ROS_INFO("distance matrix:");
    distance.print();
    ROS_INFO("terrainHazard matrix:");
    terrainHazard.print();
    std::cout << "enter a character to continue" << std::endl;
    std::cin >> temp;*/
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
                /*ROS_INFO("before value computation, i=%i j=%i",i,j);
				ROS_INFO("pheromone(i,j) = %f",pheromone(i,j));
				ROS_INFO("waypointsToPlan.at(j).sampleProb = %f",waypointsToPlan.at(j).sampleProb);
				ROS_INFO("sampleProbGain*sampleProb = %f",sampleProbGain*waypointsToPlan.at(j).sampleProb);
				ROS_INFO("pheromoneGain*pheromone(i,j) = %f",pheromoneGain*pheromone(i,j));
				ROS_INFO("distance(i,j) = %f",distance(i,j));
				ROS_INFO("distanceGain*distance(i,j) = %f",distanceGain*distance(i,j));
				ROS_INFO("terrainHazard(i,j) = %f",terrainHazard(i,j));
				ROS_INFO("terrainGain*terrainHazard(i,j) = %f",terrainGain*terrainHazard(i,j));
                ROS_INFO("notVisited[j] = %i",notVisited.at(j));*/
				computedValue =	(sampleProbGain*waypointsToPlan.at(j).sampleProb +
								pheromoneGain*pheromone(i,j) -
								distanceGain*distance(i,j) -
								terrainGain*terrainHazard(i,j));
                //ROS_INFO("after value computation, before coersion. computedValue = %f",computedValue);
				if(computedValue <= 0.0 && notVisited.at(j)==1) value.at(j) = 0.001;
				else value.at(j) = ((float)notVisited.at(j))*computedValue;
                //ROS_INFO("after value computation. value[j] = %f",value.at(j));
			}
			for(int k=0; k<numWaypointsToPlan; k++) valueSum += value.at(k);
            //ROS_INFO("valueSum = %f",valueSum);
			for(int k=0; k<numWaypointsToPlan; k++)
			{
				valueNormalized.at(k) = value.at(k)/valueSum;
				if(value.at(k)!=0.0 && valueNormalized.at(k)==0.0) valueNormalized.at(k) = 0.001; // Basement for normalized values. Probability distribution is discrete, not continuous, so there needs to be a basement (0.1%) that even the smallest probabilities get rounded up to so they are included in the distribution and not lost due to discrete rounding down
			}
			for(int k=0; k<numWaypointsToPlan; k++) valueNormalizedSum += valueNormalized.at(k);
            //ROS_INFO("valueNormalizedSum = %f",valueNormalizedSum);
			randomValue = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            //ROS_INFO("randomValue = %f",randomValue);
            //for(int z=0; z<valueNormalized.size(); z++) ROS_INFO("valueNormalized.at(%i) = %f",z,valueNormalized.at(z));
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
                    /*ROS_INFO("k = %i",k);
					ROS_INFO("value[k] = %f",value.at(k));
					ROS_INFO("valueNormalized[k] = %f",valueNormalized.at(k));
					ROS_INFO("valueNormalizedFloor = %f",valueNormalizedFloor);
					ROS_INFO("randomValue = %f",randomValue);
                    ROS_INFO("valueNormalizedCeiling = %f\n",valueNormalizedFloor + valueNormalized.at(k));*/
					if(randomValue >= valueNormalizedFloor && randomValue < (valueNormalizedFloor + valueNormalized.at(k)) && valueNormalized.at(k)!=0.0) {bestJ = k; break; ROS_WARN("found bestJ = %i",bestJ);}
					else valueNormalizedFloor += valueNormalized.at(k);
				}
			}
            //ROS_DEBUG("before pheromone increment, bestJ=%i, i=%i",bestJ,i);
            //ROS_DEBUG("pheroDepoGain/distance(i,bestJ) = %f\n", pheroDepoGain/distance(i,bestJ));
			// Deposit new pheromone
            //ROS_INFO("---------- i = %i, bestJ = %i",i,bestJ);
			pheromone(i,bestJ) += (pheroDepoGain/distance(i,bestJ) + pheroDecayValue);
			pheromone(bestJ,i) += (pheroDepoGain/distance(bestJ,i) + pheroDecayValue);
            //ROS_INFO("distance(i,bestJ) = %f",distance(bestJ,i));
            //ROS_INFO("pheroDepo = %f",(pheroDepoGain/distance(i,bestJ) + pheroDecayValue));

            //ROS_DEBUG("after pheromone increment");
			notVisited.at(bestJ) = 0;
			i = bestJ;
			j = 0;
			notVisitedSum = 0;
			for(int k=0; k<numWaypointsToPlan; k++) notVisitedSum += notVisited.at(k);
		}
		for(int m=0; m<numWaypointsToPlan; m++) // Pheromone decay and min and max coersion checks
		{
			for(int n=0; n<numWaypointsToPlan; n++)
			{
				pheromone(m,n) -= pheroDecayValue;
				if(pheromone(m,n)<0.0) pheromone(m,n) = 0.0;
				else if(pheromone(m,n)>maxPheromoneValue) pheromone(m,n) = maxPheromoneValue;
			}
		}
		//ROS_INFO("pheromone matrix, loop:");
		//pheromone.print();
		// Add some other condition for cutting off algorithm early if clear optimum is being reached
	}
	//ROS_INFO("pheromone matrix, end:");
	//pheromone.print();
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
			if((notVisited.at(j)*pheromone(i,j))>bestPheromone) {bestPheromone = pheromone(i,j); bestJ = j;/* ROS_INFO("bestPheromone found, bestJ = %i", bestJ);*/}
		}
		waypointsToTravel.at(o) = waypointsToPlan.at(bestJ);
		notVisited.at(bestJ) = 0;
		i = bestJ;
		//ROS_INFO("waypointsToTravel[%i]: x = %f  y = %f", o, waypointsToTravel.at(o).x, waypointsToTravel.at(o).y);
	}
}
