#include <robot_control/choose_region.h>

bool ChooseRegion::runProc()
{
    switch(state)
    {
    case _init_:
		procsBeingExecuted.at(procType) = true;
        execDequeEmpty = false;
        // Request info about regions
        if(reqROIClient.call(regionsOfInterestSrv)) ROS_DEBUG("regionsOfInterest service call successful");
        else ROS_ERROR("regionsOfInterest service call unsuccessful");

        // Loop through list and choose best region not yet visited
        bestROIValue = 0;
        bestROINum = 0;
        roiVisitedSum = 0;
        for(int i=0; i < regionsOfInterestSrv.response.waypointArray.size(); i++)
        {
            roiValue = (!regionsOfInterestSrv.response.waypointArray.at(i).visited)*
                        (easyProbGain*regionsOfInterestSrv.response.waypointArray.at(i).easyProb +
                        medProbGain*regionsOfInterestSrv.response.waypointArray.at(i).medProb +
                        hardProbGain*regionsOfInterestSrv.response.waypointArray.at(i).hardProb -
                        distanceGain*hypot(regionsOfInterestSrv.response.waypointArray.at(i).x - robotStatus.xPos,
                                           regionsOfInterestSrv.response.waypointArray.at(i).y - robotStatus.yPos)/* -
                        terrainGain*terrainHazard(i,j)*/) /
                        (easyProbGain + medProbGain + hardProbGain);
            ROS_DEBUG("!)!)!)!)!)!) roiValue = %i, roiNum = %i",roiValue, i);
            ROS_DEBUG("visited = %i",regionsOfInterestSrv.response.waypointArray.at(i).visited);
            if(roiValue > bestROIValue) {bestROINum = i; bestROIValue = roiValue;}
            roiVisitedSum += regionsOfInterestSrv.response.waypointArray.at(i).visited;
        }
		if(roiVisitedSum < regionsOfInterestSrv.response.waypointArray.size()) // Temp!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        {
            // Call service to drive to center of the chosen region
            numWaypointsToTravel = 1;
            clearAndResizeWTT();
            waypointsToTravel.at(0) = regionsOfInterestSrv.response.waypointArray.at(bestROINum);
            callIntermediateWaypoints();
			sendDriveGlobal(false);
            state = _exec_;
        }
		else // Also temp. Just used to keep the robot going in a loop !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        {
            for(int i=0; i<regionsOfInterestSrv.response.waypointArray.size(); i++)
            {
                modROISrv.request.setVisitedROI = true;
                modROISrv.request.visitedROIState = false;
                modROISrv.request.numVisitedROI = i;
                modROISrv.request.addNewROI = false;
                if(modROIClient.call(modROISrv)) ROS_DEBUG("modify ROI service call successful");
                else ROS_ERROR("modify ROI service call unsuccessful");
            }
            numWaypointsToTravel = 1;
            clearAndResizeWTT();
            waypointsToTravel.at(0).x = 8.0;
            waypointsToTravel.at(0).y = 0.0;
            callIntermediateWaypoints();
			sendDriveGlobal(false);
            state = _init_;
        }
        break;
    case _exec_:
        if(execDequeEmpty && execLastProcType == procType && execLastSerialNum == serialNum) state = _finish_;
        else state = _exec_;
        break;
    case _interrupt_:
		procsBeingExecuted.at(procType) = false;
		state = _exec_;
        break;
    case _finish_:
		procsBeingExecuted.at(procType) = false;
        // ************************ THIS NEEDS TO GO SOMEWHERE ELSE LATER
        modROISrv.request.setVisitedROI = true;
        modROISrv.request.visitedROIState = true;
        modROISrv.request.numVisitedROI = bestROINum;
        modROISrv.request.addNewROI = false;
        if(modROIClient.call(modROISrv)) ROS_DEBUG("modify ROI service call successful");
        else ROS_ERROR("modify ROI service call unsuccessful");
        // ********************************************
        state = _init_;
        break;
    }


}
