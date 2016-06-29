#include <robot_control/confirm_collect.h>

bool ConfirmCollect::runProc()
{
	ROS_INFO("confirmCollectState = %i", state);
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		sampleTypeMux = 0;
		//sampleTypeMux = (1 << (static_cast<uint8_t>(bestSample.type) & 255)) & 255;
		//ROS_INFO("confirmCollect sampleTypeMux = %u",sampleTypeMux);
		//sendSearch(sampleTypeMux);
		expectedSampleAngle = 0.0;
		expectedSampleDistance = distanceToGrabber - backUpDistance;
		sendDriveRel(backUpDistance, 0.0, false, 0.0, false, false);
        sendSearch(252); // 124 = b11111100 -> cached = 1; purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
		state = _exec_;
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		if(cvSamplesFoundMsg.procType==this->procType && cvSamplesFoundMsg.serialNum==this->serialNum)
		{
			computeSampleValuesWithExpectedDistance();
			if(bestSampleValue < confirmCollectValueThreshold) noSampleOnGround = true;
			else noSampleOnGround = false;
			if(noSampleOnGround)
			{
				confirmedPossession = true;
                inSearchableRegion = false;
				state = _finish_;
			}
			else
			{
				confirmedPossession = false;
				possessingSample = false;
				sendOpen();
				state = _finish_;
			}
		}
		else state = _exec_;
		break;
	case _interrupt_:
		avoidLockout = false;
		sampleDataActedUpon = true;
		sampleInCollectPosition = false;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _init_;
		break;
	case _finish_:
		avoidLockout = false;
		sampleDataActedUpon = true;
		sampleInCollectPosition = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		state = _init_;
		break;
	}
}
