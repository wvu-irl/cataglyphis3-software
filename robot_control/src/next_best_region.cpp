#include <robot_control/next_best_region.h>

bool NextBestRegion::runProc()
{
    //ROS_INFO("nextBestRegion state = %i",state);
    //ROS_INFO_THROTTLE(1, "executing nextBestRegion");
    switch(state)
    {
    case _init_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        execDequeEmpty = false;
        atHome = false;
        avoidCount = 0;
        prevAvoidCountDecXPos = robotStatus.xPos;
        prevAvoidCountDecYPos = robotStatus.yPos;
        examineCount = 0;
        confirmCollectFailedCount = 0;
        // Delete search local map in case region was exited without a successful sample collection
        searchMapSrv.request.createMap = false;
        searchMapSrv.request.deleteMap = true;
        if(searchMapClient.call(searchMapSrv)) ROS_DEBUG("searchMap service call successful");
        else ROS_ERROR("searchMap service call unsuccessful");
        roiKeyframed = false;
        // Request info about regions
        if(reqROIClient.call(regionsOfInterestSrv)) ROS_DEBUG("regionsOfInterest service call successful");
        else ROS_ERROR("regionsOfInterest service call unsuccessful");
        // Loop through list of ROIs and compute terrainHazard value along path from current location to each ROI center
        terrainHazard.resize(regionsOfInterestSrv.response.ROIList.size());
        for(int i=0; i < regionsOfInterestSrv.response.ROIList.size(); i++)
        {
            globalMapPathHazardsSrv.request.xStart = robotStatus.xPos;
            globalMapPathHazardsSrv.request.yStart = robotStatus.yPos;
            globalMapPathHazardsSrv.request.xEnd = regionsOfInterestSrv.response.ROIList.at(i).x;
            globalMapPathHazardsSrv.request.yEnd = regionsOfInterestSrv.response.ROIList.at(i).y;
            globalMapPathHazardsSrv.request.width = hazardCorridorWidth;
            if(globalMapPathHazardsClient.call(globalMapPathHazardsSrv)) ROS_DEBUG("globalMapPathHazardsSrv call successful");
            else ROS_ERROR("globalMapPathHazardsSrv call unsuccessful");
            /*terrainHazard.at(i) = numHazardsPerDistanceToTerrainHazardGain*(float)globalMapPathHazardsSrv.response.numHazards /
                                  hypot(regionsOfInterestSrv.response.ROIList.at(i).x - robotStatus.xPos,
                                        regionsOfInterestSrv.response.ROIList.at(i).y - robotStatus.yPos);*/
            terrainHazard.at(i) = globalMapPathHazardsSrv.response.hazardValue;
            if(terrainHazard.at(i) < 0.0) terrainHazard.at(i) = 0.0;
        }
        // Loop through list of ROIs and choose best region not yet searched
        bestROIValue = 0;
        bestROINum = 0;
        roiSearchedSum = 0;
		for(int i=0; i < regionsOfInterestSrv.response.ROIList.size(); i++)
        {
            roiValue = (sampleProbGain*regionsOfInterestSrv.response.ROIList.at(i).sampleProb +
                        sampleSigGain*regionsOfInterestSrv.response.ROIList.at(i).sampleSig -
						distanceGain*hypot(regionsOfInterestSrv.response.ROIList.at(i).x - robotStatus.xPos,
                                           regionsOfInterestSrv.response.ROIList.at(i).y - robotStatus.yPos) -
						terrainGain*terrainHazard.at(i));
            ROS_INFO("!)!)!)!)!)!) roiValue before coersion = %f, roiNum = %i",roiValue, i);
			ROS_INFO("searched = %i",regionsOfInterestSrv.response.ROIList.at(i).searched);
            if(roiValue <= 0.0 && !regionsOfInterestSrv.response.ROIList.at(i).searched) roiValue = 0.001;
            else roiValue = (!regionsOfInterestSrv.response.ROIList.at(i).searched)*roiValue;
            ROS_INFO("!(!(!(!(!(!( roiValue after coersion = %f, roiNum = %i",roiValue, i);
            if(roiValue > bestROIValue) {bestROINum = i; bestROIValue = roiValue;}
			roiSearchedSum += regionsOfInterestSrv.response.ROIList.at(i).searched;
        }
        if(roiSearchedSum < regionsOfInterestSrv.response.ROIList.size())
        {
            // Call service to drive to center of the chosen region

            /*numWaypointsToTravel = 1;
            clearAndResizeWTT();
            waypointsToTravel.at(0).x = regionsOfInterestSrv.response.ROIList.at(bestROINum).x;
            waypointsToTravel.at(0).y = regionsOfInterestSrv.response.ROIList.at(bestROINum).y;
            waypointsToTravel.at(0).searchable = false; // !!!!! NEEDS TO BE TRUE to search
            callIntermediateWaypoints();
            sendDriveAndSearch(252); // 252 = b11111100 -> cached = 1; purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
            sendWait(10.0);*/

            ROS_INFO("NEXT BEST REGION: bestROINum = %i",bestROINum);
            numWaypointsToTravel = 2;
            clearAndResizeWTT();
            angleToROI = atan2(regionsOfInterestSrv.response.ROIList.at(bestROINum).y - robotStatus.yPos, regionsOfInterestSrv.response.ROIList.at(bestROINum).x - robotStatus.xPos); // Radians
            waypointsToTravel.at(0).x = regionsOfInterestSrv.response.ROIList.at(bestROINum).x - distanceShortOfROI*cos(angleToROI);
            waypointsToTravel.at(0).y = regionsOfInterestSrv.response.ROIList.at(bestROINum).y - distanceShortOfROI*sin(angleToROI);
            waypointsToTravel.at(0).searchable = true; // !!!!! NEEDS TO BE TRUE to search
            waypointsToTravel.at(1).x = regionsOfInterestSrv.response.ROIList.at(bestROINum).x;
            waypointsToTravel.at(1).y = regionsOfInterestSrv.response.ROIList.at(bestROINum).y;
            waypointsToTravel.at(1).searchable = true; // !!!!! NEEDS TO BE TRUE to search
            callIntermediateWaypoints();
            sendDriveAndSearch(252); // 252 = b11111100 -> cached = 1; purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
            //sendWait(10.0);

            currentROIIndex = bestROINum;
            allocatedROITime = 60.0;//270.0; // sec == 4.5 min; implement specific times in ROIs as properties
            tempGoHome = false;
            state = _exec_;
        }
        else // !!! Temporary condition. End mission if all ROIs have been searched
        {
            /*for(int i=0; i<regionsOfInterestSrv.response.ROIList.size(); i++)
            {
				modROISrv.request.setSearchedROI = true;
				modROISrv.request.searchedROIState = false;
                modROISrv.request.modROIIndex = i;
                modROISrv.request.addNewROI = false;
                modROISrv.request.deleteROI = false;
                if(modROIClient.call(modROISrv)) ROS_DEBUG("modify ROI service call successful");
                else ROS_ERROR("modify ROI service call unsuccessful");
            }
            numWaypointsToTravel = 1;
            clearAndResizeWTT();
            waypointsToTravel.at(0).x = 5.0;
            waypointsToTravel.at(0).y = 0.0;
            waypointsToTravel.at(0).searchable = false;
            callIntermediateWaypoints();
            sendDriveGlobal(false, false, 0.0);
			procsBeingExecuted[procType] = false;
            tempGoHome = true;*/
            missionEnded = true;
            state = _finish_;
        }
        computeDriveSpeeds();
        break;
    case _exec_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        serviceAvoidCounterDecrement();
        if((cvSamplesFoundMsg.procType==this->procType && cvSamplesFoundMsg.serialNum==this->serialNum) || possibleSample || definiteSample) state = _finish_;
        else state = _exec_;
        /*if(execLastProcType == procType && execLastSerialNum == serialNum) state = _finish_;
        else state = _exec_;*/

        /*if(waypointsToTravel.at(0).searchable)
        {
            ROS_INFO("searchable case");
            if((cvSamplesFoundMsg.procType==this->procType && cvSamplesFoundMsg.serialNum==this->serialNum) || possibleSample || definiteSample) state = _finish_;
            else state = _exec_;
        }
        else
        {
            ROS_INFO("non-searchable case");
            if(execLastProcType == procType && execLastSerialNum == serialNum) state = _finish_;
            else state = _exec_;
        }*/
        break;
    case _interrupt_:
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _exec_;
        break;
    case _finish_:
        if(execLastProcType != procType || execLastSerialNum != serialNum) sendDequeClearAll();
        inSearchableRegion = true;
        /*if(waypointsToTravel.at(0).searchable) inSearchableRegion = true;
        else
        {
            // ************************ THIS IS TEMPORARY TO ALLOW FOR DRIVING WITHOUT SEARCHING
            if(!tempGoHome)
            {
                modROISrv.request.setSearchedROI = true;
                modROISrv.request.searchedROIState = true;
                modROISrv.request.modROIIndex = currentROIIndex;
                modROISrv.request.addNewROI = false;
                modROISrv.request.deleteROI = false;
                if(modROIClient.call(modROISrv)) ROS_DEBUG("modify ROI service call successful");
                else ROS_ERROR("modify ROI service call unsuccessful");
            }
            // ********************************************
        }*/
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        state = _init_;
        break;
    }


}
