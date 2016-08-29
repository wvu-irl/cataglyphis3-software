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
        if(collisionMsg.collision==2) {avoidCount += fullyBlockedAvoidCountIncrement; ROS_INFO("fully blocked avoid"); voiceSay->call("fully blocked avoid");}
        else {avoidCount++; ROS_INFO("normal avoid");}
        if(static_cast<ACTION_TYPE_T>(execInfoMsg.actionDeque[interruptedAvoid]) == _driveGlobal) maxAvoidCount = execInfoMsg.actionInt1[interruptedAvoid];
        else {maxAvoidCount = maxNormalWaypointAvoidCount; ROS_WARN("avoided on something that's not driveGlobal, set max avoid to default");}
        //ROS_INFO("avoidCount = %u",avoidCount);
        //ROS_INFO("maxAvoidCount = %u",maxAvoidCount);
        //ROS_INFO("interruptedAvoid = %i",interruptedAvoid);
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
                if(currentROIIndex==0) modROISrv.request.sampleProb = 0.0;
                else modROISrv.request.sampleProb = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).sampleProb*giveUpROIFromAvoidNewSampleProbMultiplier;
                modROISrv.request.sampleSig = regionsOfInterestSrv.response.ROIList.at(currentROIIndex).sampleSig;
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
                giveUpROI = true;
                sendDequeClearAll();
                ROS_INFO("AVOID gave up ROI number %i",currentROIIndex);
                voiceSay->call("give up r o i");
                break;
            }
            if(!execInfoMsg.actionBool3[interruptedAvoid] && (interruptedAvoid || interruptedEmergencyEscape))
            {
                ROS_INFO("clear two actions from actionDeque");
                sendDequeClearFront(); // Send once to clear the avoid drive action
                sendDequeClearFront(); // And send again to clear prev proc's drive action as well
                dequeClearFront = false;
                interruptedEmergencyEscape = false;
                interruptedAvoid = 0;
                avoidCount = 0;
                procsBeingExecuted[procType] = false;
                procsToExecute[procType] = false;
                state = _init_;
                break;
            }
            else if(!execInfoMsg.actionBool3[interruptedAvoid] != !(interruptedAvoid || interruptedEmergencyEscape))
            {
                ROS_INFO("clear one action from actionDeque");
                sendDequeClearFront(); // Send once to clear the current drive action
                dequeClearFront = false;
                interruptedEmergencyEscape = false;
                interruptedAvoid = 0;
                avoidCount = 0;
                procsBeingExecuted[procType] = false;
                procsToExecute[procType] = false;
                state = _init_;
                break;
            }
        }
        if(interruptedAvoid || interruptedEmergencyEscape)
        {
            sendDequeClearFront(); // Send once to clear the avoid or emergency escape drive action
            interruptedAvoid = 0;
            interruptedEmergencyEscape = false;
        }
        computeDriveSpeeds();
        sendDriveRel(collisionMsg.distance_to_drive, collisionMsg.angle_to_drive, false, 0.0, true, false);
        state = _exec_;
        resetQueueEmptyCondition();
		break;
	case _exec_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        if(!execInfoMsg.stopFlag && !execInfoMsg.turnFlag && execInfoMsg.actionProcType.at(0)==procType) avoidLockout = false;
        if((execLastProcType == procType && execLastSerialNum == serialNum) || queueEmptyTimedOut) state = _finish_;
		else state = _exec_;
        serviceQueueEmptyCondition();
		break;
	case _interrupt_:
        procsBeingExecuted[procType] = true;
		procsToInterrupt[procType] = false;
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
