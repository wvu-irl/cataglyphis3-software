#include <robot_control/initialize.h>

bool Initialize::runProc()
{
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		procsToResume[procType] = false;
		sendWait(initWaitTime, false);
		sendOpen(); // Make sure the grabber is open initially
		sendDriveRel(driveOffPlatformDistance, 0.0, false, 0.0, false, false);
		step = _drivingOffPlatform;
		state = _exec_;
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		procsToResume[procType] = false;
		switch(step)
		{
		case _drivingOffPlatform:
			if(execLastProcType == procType && execLastSerialNum == serialNum)
			{
				biasRemovalTimedOut = false;
				timers[_biasRemovalActionTimeoutTimer_]->start();
				navControlSrv.request.runBiasRemoval = true;
				if(navControlClient.call(navControlSrv)) ROS_DEBUG("navFilterControlService call successful");
				else ROS_ERROR("navFilterControlService call unsuccessful");
				step = _firstBiasRemoval;
			}
			else
			{
				step = _drivingOffPlatform;
				state = _exec_;
			}
			break;
		case _firstBiasRemoval:
			if(navStatus!=0 || biasRemovalTimedOut)
			{
				timers[_biasRemovalActionTimeoutTimer_]->stop();
				timers[_biasRemovalTimer_]->stop();
				timers[_biasRemovalTimer_]->start();
				biasRemovalTimedOut = false;
				performBiasRemoval = false;
				state = _finish_;
			}
			else
			{
				step = _firstBiasRemoval;
				state = _exec_;
			}
			break;
		}

		break;
	case _interrupt_:
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _finish_;
		break;
	case _finish_:
		startSLAM = true;
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		procsToResume[procType] = false;
		initialized = true;
		state = _finish_;
		break;
	}
}
