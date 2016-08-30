#include <robot_control/collect.h>

bool Collect::runProc()
{
    //ROS_INFO("collectState = %i", state);
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        if(sideOffsetGrab) {sendDriveRel(0.0, sideGrabAngleOffset, false, 0.0, false, false); ROS_INFO("side offset grab");}
        else {sendDriveRel(blindDriveDistance, 0.0, false, 0.0, false, true); ROS_INFO("normal straight grab");}
		sendGrab();
        computeDriveSpeeds();
		state = _exec_;
        resetQueueEmptyCondition();
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        if((execLastProcType == procType && execLastSerialNum == serialNum) || queueEmptyTimedOut) state = _finish_;
		else state = _exec_;
        if(grabberStatusMsg.dropFailed || grabberStatusMsg.slidesFailed) {dropOrSlidesFailed = true; ROS_WARN("collect drop or slides failed");}
        else dropOrSlidesFailed = false;
        serviceQueueEmptyCondition();
		break;
	case _interrupt_:
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _init_;
		break;
	case _finish_:
        if(dropOrSlidesFailed)
        {
            avoidLockout = false;
            possessingSample = false;
            sideOffsetGrab = false;
            performReorient = true;
            sampleInCollectPosition = false;
            sendDriveRel(failedBackUpDistance, 0.0, false, 0.0, false, false);
        }
        else
        {
            avoidLockout = true;
            possessingSample = true;
            sideOffsetGrab = false;
        }
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		state = _init_;
		break;
	}
}
