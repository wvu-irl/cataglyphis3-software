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
        if(collisionMsg.collision==2) {avoidCount += fullyBlockedAvoidCountIncrement; ROS_INFO("fully blocked avoid");}
        else {avoidCount++; ROS_INFO("normal avoid");}
        if(static_cast<ACTION_TYPE_T>(execInfoMsg.actionDeque[interruptedAvoid]) == _driveGlobal) maxAvoidCount = execInfoMsg.actionInt1[interruptedAvoid];
        else {maxAvoidCount = maxNormalWaypointAvoidCount; ROS_WARN("avoided on something that's not driveGlobal, set max avoid to default");}
        ROS_INFO("avoidCount = %u",avoidCount);
        ROS_INFO("maxAvoidCount = %u",maxAvoidCount);
        /*if(static_cast<ACTION_TYPE_T>(execInfoMsg.actionDeque[interruptedAvoid]) == _driveGlobal && execInfoMsg.actionBool4[interruptedAvoid])
            maxAvoidCount = maxROIWaypointAvoidCount;
        else maxAvoidCount = maxNormalWaypointAvoidCount;*/
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
                interruptedEmergencyEscape = false;
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
                interruptedEmergencyEscape = false;
                avoidCount = 0;
                procsBeingExecuted[procType] = false;
                procsToExecute[procType] = false;
                state = _init_;
                break;
            }
        }
        computeDriveSpeeds();
        sendDriveRel(collisionMsg.distance_to_drive, collisionMsg.angle_to_drive, false, 0.0, true, false);
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
