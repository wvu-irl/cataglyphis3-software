#include <robot_control/deposit_sample.h>

bool DepositSample::runProc()
{
    //ROS_INFO("depositSampleState = %i", state);
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        dropFailed = false;
        driveSpeedsMsg.vMax = slowVMax;
        driveSpeedsMsg.rMax = defaultRMax;
        driveSpeedsMsgPrev.vMax = slowVMax;
        driveSpeedsMsgPrev.rMax = defaultRMax;
        driveSpeedsPub.publish(driveSpeedsMsg);
		samplesCollected++;
		if(samplesCollected<MAX_SAMPLES)
		{
			sendDrop();
            sendDriveRel(-3.0, 0.0, false, 0.0, false, false);
			missionEnded = false;
			state = _exec_;
            voiceSay->call("I'll be back");
		}
		else
		{
			missionEnded = true;
			state = _finish_;
            voiceSay->call("mission ended. show me the money");
		}
        resetQueueEmptyCondition();
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        if((execLastProcType == procType && execLastSerialNum == serialNum) || queueEmptyTimedOut) state = _finish_;
		else state = _exec_;
        if(grabberStatusMsg.dropFailed) dropFailed = true;
        else dropFailed = false;
        serviceQueueEmptyCondition();
		break;
	case _interrupt_:
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _init_;
		break;
	case _finish_:
        if(dropFailed)
        {
            avoidLockout = true;
            possessingSample = true;
            confirmedPossession = true;
            atHome = true;
            inDepositPosition = false;
            startSLAM = false;
        }
        else
        {
            avoidLockout = false;
            possessingSample = false;
            confirmedPossession = false;
            atHome = false;
            inDepositPosition = false;
            possibleSample = false;
            definiteSample = false;
            sampleDataActedUpon = false;
            sampleInCollectPosition = false;
            startSLAM = true;
        }
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		state = _init_;
		break;
	}
}
