#include <robot_control/safe_mode.h>

SafeMode::SafeMode()
{
	srand(time(NULL));
}

bool SafeMode::runProc()
{
	switch(state)
	{
	case _init_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		computeDriveSpeeds();
		avoidCount = 0;
		driveDistance = maxDriveDistance*(static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
		if(driveDistance < minDriveDistance) driveDistance = minDriveDistance;
		turnAngle = maxTurnAngle*(static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
		sendDriveRel(driveDistance, turnAngle, false, 0.0, false);
		sendWait(lidarUpdateWaitTime);
		state = _exec_;
		break;
	case _exec_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		computeDriveSpeeds();
		if(execLastProcType == procType && execLastSerialNum == serialNum) state = _finish_;
		else state = _exec_;
		break;
	case _interrupt_:
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		sendDequeClearAll();
		state = _init_;
		break;
	case _finish_:
		avoidLockout = false;
		if(robotStatus.homingUpdated)
		{
			homingUpdatedFailedCount = 0;
			performHoming = true;
			homingUpdateFailed = false;
			useDeadReckoning = false;
			performSafeMode = false;
		}
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		state = _init_;
		break;
	}
}
