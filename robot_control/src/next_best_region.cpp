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
        if(timers[_roiTimer_]->running) timers[_roiTimer_]->stop();
        roiTimeExpired = false;
        if(timers[_roiOvertimeTimer_]->running) timers[_roiOvertimeTimer_]->stop();
        roiOvertimeExpired = false;
        avoidCount = 0;
        prevAvoidCountDecXPos = robotStatus.xPos;
        prevAvoidCountDecYPos = robotStatus.yPos;
        examineCount = 0;
        confirmCollectFailedCount = 0;
        clearSampleHistory();
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
        for(int i=0; i < regionsOfInterestSrv.response.ROIList.size(); i++)
        {
            if(regionsOfInterestSrv.response.ROIList.at(i).sampleProb > 0.0)
            {
                bestROINum = i;
                break;
            }
        }
        roiHardLockoutSum = 0;
		for(int i=0; i < regionsOfInterestSrv.response.ROIList.size(); i++)
        {
            roiValue = sampleProbGain*regionsOfInterestSrv.response.ROIList.at(i).sampleProb +
                        sampleSigGain*regionsOfInterestSrv.response.ROIList.at(i).sampleSig -
                        distanceGain*(hypot(regionsOfInterestSrv.response.ROIList.at(i).x - robotStatus.xPos,
                                           regionsOfInterestSrv.response.ROIList.at(i).y - robotStatus.yPos)
                                      + hypot(regionsOfInterestSrv.response.ROIList.at(i).x, regionsOfInterestSrv.response.ROIList.at(i).y)) -
                        terrainGain*terrainHazard.at(i) -
                        riskGain*regionsOfInterestSrv.response.ROIList.at(i).highRisk;
            ROS_INFO("!)!)!)!)!)!) roiValue before coersion = %f, roiNum = %i",roiValue, i);
            ROS_INFO("hardLockout = %i",regionsOfInterestSrv.response.ROIList.at(i).hardLockout);
            ROS_INFO("prevROIIndex = %i",prevROIIndex);
            if(roiValue <= 0.0 && !regionsOfInterestSrv.response.ROIList.at(i).hardLockout && i != prevROIIndex && regionsOfInterestSrv.response.ROIList.at(i).sampleProb!=0.0)
            {
                roiValue += negValueIncrement;
                if(roiValue > maxCoercedNegValue) roiValue = maxCoercedNegValue;
            }
            else if(regionsOfInterestSrv.response.ROIList.at(i).hardLockout || i == prevROIIndex || regionsOfInterestSrv.response.ROIList.at(i).sampleProb==0.0)
                roiValue = 0.0;
            ROS_INFO("!(!(!(!(!(!( roiValue after coersion = %f, roiNum = %i",roiValue, i);
            if(roiValue > bestROIValue) {bestROINum = i; bestROIValue = roiValue;}
            roiHardLockoutSum += regionsOfInterestSrv.response.ROIList.at(i).hardLockout;
        }
        if(roiHardLockoutSum < regionsOfInterestSrv.response.ROIList.size())
        {
            // Call service to drive to center of the chosen region

            /*numWaypointsToTravel = 1;
            clearAndResizeWTT();
            waypointsToTravel.at(0).x = regionsOfInterestSrv.response.ROIList.at(bestROINum).x;
            waypointsToTravel.at(0).y = regionsOfInterestSrv.response.ROIList.at(bestROINum).y;
            waypointsToTravel.at(0).searchable = false; // !!!!! NEEDS TO BE TRUE to search
            callIntermediateWaypoints();
            sendDriveAndSearch(252); // 252 = b11111100 -> cached = 1; purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
            sendWait(10.0, false);*/

            ROS_INFO("NEXT BEST REGION: bestROINum = %i",bestROINum);
            numWaypointsToTravel = 2;
            clearAndResizeWTT();
            angleToROI = atan2(regionsOfInterestSrv.response.ROIList.at(bestROINum).y - robotStatus.yPos, regionsOfInterestSrv.response.ROIList.at(bestROINum).x - robotStatus.xPos); // Radians
            waypointsToTravel.at(0).x = regionsOfInterestSrv.response.ROIList.at(bestROINum).x - distanceShortOfROI*cos(angleToROI);
            waypointsToTravel.at(0).y = regionsOfInterestSrv.response.ROIList.at(bestROINum).y - distanceShortOfROI*sin(angleToROI);
            waypointsToTravel.at(0).searchable = true; // !!!!! NEEDS TO BE TRUE to search
            waypointsToTravel.at(0).unskippable = false;
            waypointsToTravel.at(0).roiWaypoint = true;
            waypointsToTravel.at(0).maxAvoids = maxROIWaypointAvoidCount;
            waypointsToTravel.at(0).whiteProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).whiteProb;
            waypointsToTravel.at(0).silverProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).silverProb;
            waypointsToTravel.at(0).blueOrPurpleProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).blueOrPurpleProb;
            waypointsToTravel.at(0).pinkProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).pinkProb;
            waypointsToTravel.at(0).redProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).redProb;
            waypointsToTravel.at(0).orangeProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).orangeProb;
            waypointsToTravel.at(0).yellowProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).yellowProb;
            waypointsToTravel.at(1).x = regionsOfInterestSrv.response.ROIList.at(bestROINum).x;
            waypointsToTravel.at(1).y = regionsOfInterestSrv.response.ROIList.at(bestROINum).y;
            waypointsToTravel.at(1).searchable = true; // !!!!! NEEDS TO BE TRUE to search
            waypointsToTravel.at(1).unskippable = false;
            waypointsToTravel.at(1).roiWaypoint = true;
            waypointsToTravel.at(1).maxAvoids = maxROIWaypointAvoidCount;
            waypointsToTravel.at(1).whiteProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).whiteProb;
            waypointsToTravel.at(1).silverProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).silverProb;
            waypointsToTravel.at(1).blueOrPurpleProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).blueOrPurpleProb;
            waypointsToTravel.at(1).pinkProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).pinkProb;
            waypointsToTravel.at(1).redProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).redProb;
            waypointsToTravel.at(1).orangeProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).orangeProb;
            waypointsToTravel.at(1).yellowProb = regionsOfInterestSrv.response.ROIList.at(bestROINum).yellowProb;
            callIntermediateWaypoints();
            sendDriveAndSearch();
            //sendWait(10.0, false);

            currentROIIndex = bestROINum;
            allocatedROITime = regionsOfInterestSrv.response.ROIList.at(bestROINum).allocatedTime;
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
            missionEnded = true; // !!! should be super safe mode
            state = _finish_;
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
        if(searchEnded() || possibleSample || definiteSample || giveUpROI || queueEmptyTimedOut) state = _finish_;
        else state = _exec_;
        serviceQueueEmptyCondition();
        break;
    case _interrupt_:
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _exec_;
        break;
    case _finish_:
        if(execLastProcType != procType || execLastSerialNum != serialNum) sendDequeClearAll();
        distanceToRegion = hypot(regionsOfInterestSrv.response.ROIList.at(currentROIIndex).x - robotStatus.xPos,
                                 regionsOfInterestSrv.response.ROIList.at(currentROIIndex).y - robotStatus.yPos);
        if(giveUpROI || (distanceToRegion > maxDistanceToSearchRegion)) inSearchableRegion = false;
        else inSearchableRegion = true;
        if(distanceToRegion > maxDistanceToSearchRegion) prevROIIndex = 99;
        else prevROIIndex = currentROIIndex;
        giveUpROI = false;
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
