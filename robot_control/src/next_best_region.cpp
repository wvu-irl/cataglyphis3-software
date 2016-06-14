#include <robot_control/next_best_region.h>

bool NextBestRegion::runProc()
{
	ROS_INFO("nextBestRegion state = %i",state);
    switch(state)
    {
    case _init_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        execDequeEmpty = false;
        // Request info about regions
        if(reqROIClient.call(regionsOfInterestSrv)) ROS_DEBUG("regionsOfInterest service call successful");
        else ROS_ERROR("regionsOfInterest service call unsuccessful");

		// Loop through list and choose best region not yet searched
        bestROIValue = 0;
        bestROINum = 0;
		roiSearchedSum = 0;
		for(int i=0; i < regionsOfInterestSrv.response.ROIList.size(); i++)
        {
			roiValue = (!regionsOfInterestSrv.response.ROIList.at(i).searched)*
						(purpleProbGain*regionsOfInterestSrv.response.ROIList.at(i).purpleProb +
						redProbGain*regionsOfInterestSrv.response.ROIList.at(i).redProb +
						blueProbGain*regionsOfInterestSrv.response.ROIList.at(i).blueProb +
						silverProbGain*regionsOfInterestSrv.response.ROIList.at(i).silverProb +
						brassProbGain*regionsOfInterestSrv.response.ROIList.at(i).brassProb -
						distanceGain*hypot(regionsOfInterestSrv.response.ROIList.at(i).x - robotStatus.xPos,
										   regionsOfInterestSrv.response.ROIList.at(i).y - robotStatus.yPos)/* -
						terrainGain*terrainHazard(i,j)*/) /
						(purpleProbGain + redProbGain + blueProbGain + silverProbGain + brassProbGain);
            ROS_DEBUG("!)!)!)!)!)!) roiValue = %i, roiNum = %i",roiValue, i);
			ROS_DEBUG("searched = %i",regionsOfInterestSrv.response.ROIList.at(i).searched);
            if(roiValue > bestROIValue) {bestROINum = i; bestROIValue = roiValue;}
			roiSearchedSum += regionsOfInterestSrv.response.ROIList.at(i).searched;
        }
		if(roiSearchedSum < regionsOfInterestSrv.response.ROIList.size()) // Temp!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        {
            // Call service to drive to center of the chosen region
            numWaypointsToTravel = 1;
            clearAndResizeWTT();
			waypointsToTravel.at(0).x = regionsOfInterestSrv.response.ROIList.at(bestROINum).x;
			waypointsToTravel.at(0).y = regionsOfInterestSrv.response.ROIList.at(bestROINum).y;
            waypointsToTravel.at(0).searchable = false; // !!!!! NEEDS TO BE TRUE to search
            callIntermediateWaypoints();
			//sendDriveGlobal(false, false);
			sendDriveAndSearch(124); // 124 = b1111100 -> purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
            state = _exec_;
        }
		else // Also temp. Just used to keep the robot going in a loop !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        {
			for(int i=0; i<regionsOfInterestSrv.response.ROIList.size(); i++)
            {
				modROISrv.request.setSearchedROI = true;
				modROISrv.request.searchedROIState = false;
				modROISrv.request.numSearchedROI = i;
                modROISrv.request.addNewROI = false;
                if(modROIClient.call(modROISrv)) ROS_DEBUG("modify ROI service call successful");
                else ROS_ERROR("modify ROI service call unsuccessful");
            }
            numWaypointsToTravel = 1;
            clearAndResizeWTT();
            waypointsToTravel.at(0).x = 8.0;
            waypointsToTravel.at(0).y = 0.0;
            waypointsToTravel.at(0).searchable = false;
            callIntermediateWaypoints();
			sendDriveGlobal(false, false);
			procsBeingExecuted[procType] = false;
            state = _init_;
        }
        break;
    case _exec_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		//if(execDequeEmpty && execLastProcType == procType && execLastSerialNum == serialNum) state = _finish_;
        if(waypointsToTravel.at(0).searchable)
        {
            if(cvSamplesFoundMsg.procType==this->procType && cvSamplesFoundMsg.serialNum==this->serialNum) state = _finish_;
            else state = _exec_;
        }
        else
        {
            if(execLastProcType == procType && execLastSerialNum == serialNum) state = _finish_;
            else state = _exec_;
        }
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
        // ************************ THIS NEEDS TO GO SOMEWHERE ELSE LATER
		modROISrv.request.setSearchedROI = true;
		modROISrv.request.searchedROIState = true;
		modROISrv.request.numSearchedROI = bestROINum;
        modROISrv.request.addNewROI = false;
        if(modROIClient.call(modROISrv)) ROS_DEBUG("modify ROI service call successful");
        else ROS_ERROR("modify ROI service call unsuccessful");
        // ********************************************
        state = _init_;
        break;
    }


}
