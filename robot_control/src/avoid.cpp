#include <robot_control/avoid.h>

Avoid::Avoid()
{
    interruptedAvoid = 0;
    interruptedEmergencyEscape = false;
}

bool Avoid::runProc()
{
    //ROS_INFO("avoid state = %i",state);
	switch(state)
	{
	case _init_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        avoidCount++;
        if(static_cast<ACTION_TYPE_T>(execInfoMsg.actionDeque[interruptedAvoid]) == _driveGlobal && execInfoMsg.actionBool4[interruptedAvoid])
            maxAvoidCount = maxROIWaypointAvoidCount;
        else maxAvoidCount = maxNormalWaypointAvoidCount;
        if(avoidCount > maxAvoidCount)
        {
            //ROS_INFO("avoid count limit reached");
            if(static_cast<ACTION_TYPE_T>(execInfoMsg.actionDeque[interruptedAvoid]) == _driveGlobal && execInfoMsg.actionBool4[interruptedAvoid])
            {
                // Give up ROI
                modROISrv.request.setHardLockoutROI = false;
                modROISrv.request.hardLockoutROIState = false;
                modROISrv.request.modROIIndex = currentROIIndex;
                modROISrv.request.setSampleProps = true;
                modROISrv.request.sampleProb = giveUpROIFromAvoidNewSampleProb;
                modROISrv.request.sampleSig = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).sampleSig;
                modROISrv.request.addNewROI = false;
                modROISrv.request.editGroup = false;
                if(modROIClient.call(modROISrv)) ROS_DEBUG("modify ROI service call successful");
                else ROS_ERROR("modify ROI service call unsuccessful");
                giveUpROI = true;
                sendDequeClearAll();
                ROS_INFO("AVOID gave up ROI number %i",currentROIIndex);
            }
            if((dequeClearFront && static_cast<ACTION_TYPE_T>(execInfoMsg.actionDeque[interruptedAvoid]) == _driveGlobal && !execInfoMsg.actionBool3[interruptedAvoid]) ||
                    (dequeClearFront && static_cast<ACTION_TYPE_T>(execInfoMsg.actionDeque[interruptedAvoid]) != _driveGlobal && !execInfoMsg.actionBool3[interruptedAvoid]))
            {
                sendDequeClearFront(); // Send once to clear the avoid drive action
                sendDequeClearFront(); // And send again to clear prev proc's drive action as well
                dequeClearFront = false;
                avoidCount = 0;
                procsBeingExecuted[procType] = false;
                procsToExecute[procType] = false;
                state = _init_;
                break;
            }
            else if((dequeClearFront && static_cast<ACTION_TYPE_T>(execInfoMsg.actionDeque[interruptedAvoid]) == _driveGlobal && execInfoMsg.actionBool3[interruptedAvoid]) ||
                    (!dequeClearFront && static_cast<ACTION_TYPE_T>(execInfoMsg.actionDeque[interruptedAvoid]) != _driveGlobal && !execInfoMsg.actionBool3[interruptedAvoid]) ||
                    (!dequeClearFront && static_cast<ACTION_TYPE_T>(execInfoMsg.actionDeque[interruptedAvoid]) != _driveGlobal && !execInfoMsg.actionBool3[interruptedAvoid]) ||
                    interruptedEmergencyEscape)
            {
                sendDequeClearFront(); // Send once to clear the avoid drive action
                dequeClearFront = false;
                avoidCount = 0;
                procsBeingExecuted[procType] = false;
                procsToExecute[procType] = false;
                state = _init_;
                break;
            }
        }
        computeDriveSpeeds();
		intermediateWaypointsSrv.request.collision = collisionMsg.collision;
		intermediateWaypointsSrv.request.collisionDistance = collisionMsg.distance_to_collision;
        intermediateWaypointsSrv.request.start_x = robotStatus.xPos;
        intermediateWaypointsSrv.request.start_y = robotStatus.yPos;
		intermediateWaypointsSrv.request.current_heading = robotStatus.heading;
        intermediateWaypointsSrv.request.waypointArrayIn.resize(1);
		if(execInfoMsg.actionDeque[0]==_driveGlobal)
		{
            intermediateWaypointsSrv.request.waypointArrayIn.at(0).x = execInfoMsg.actionFloat1[0];
            intermediateWaypointsSrv.request.waypointArrayIn.at(0).y = execInfoMsg.actionFloat2[0];
		}
		else // _driveRelative // Else if? Don't want undefined behavior, but should not happen for anything other than driveGlobal and driveRelative
		{
            intermediateWaypointsSrv.request.waypointArrayIn.at(0).x = robotStatus.xPos + execInfoMsg.actionFloat1[0]*cos(execInfoMsg.actionFloat2[0]*DEG2RAD);
            intermediateWaypointsSrv.request.waypointArrayIn.at(0).y = robotStatus.yPos + execInfoMsg.actionFloat1[0]*sin(execInfoMsg.actionFloat2[0]*DEG2RAD);
		}
		if(intermediateWaypointsClient.call(intermediateWaypointsSrv)) ROS_DEBUG("intermediateWaypoints service call successful");
		else ROS_ERROR("intermediateWaypoints service call unsuccessful");
        if(intermediateWaypointsSrv.response.waypointArrayOut.size() > 0)
		{
            numWaypointsToTravel = intermediateWaypointsSrv.response.waypointArrayOut.size();
			clearAndResizeWTT();
            for(int i=0; i<numWaypointsToTravel; i++) waypointsToTravel.at(i) = intermediateWaypointsSrv.response.waypointArrayOut.at(numWaypointsToTravel-1-i);
            if(dequeClearFront || interruptedEmergencyEscape)
            {
                sendDequeClearFront();
                dequeClearFront = false;
                interruptedEmergencyEscape = false;
                ROS_INFO("avoid dequeClearFront or interruptedEmergencyEscape true");
            }
            sendDriveGlobal(true, false, 0.0);
            //ROS_INFO("avoid sendDriveGlobal(front)");

		}
		state = _exec_;
		break;
	case _exec_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        if(!execInfoMsg.stopFlag && !execInfoMsg.turnFlag && execInfoMsg.actionProcType.at(0)==procType) avoidLockout = false;
		if(execLastProcType == procType && execLastSerialNum == serialNum) state = _finish_;
		else state = _exec_;
		break;
	case _interrupt_:
        procsBeingExecuted[procType] = true;
		procsToInterrupt[procType] = false;
        dequeClearFront = true;
		state = _init_;
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
