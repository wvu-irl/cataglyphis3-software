#include <robot_control/sos_mode.h>

SosMode::SosMode()
{
	srand(time(NULL));
}

bool SosMode::runProc()
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
		turnAngle = maxTurnAngle*(static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
		sendDriveRel(0.0, turnAngle, false, 0.0, false, false);
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
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		procsToResume[procType] = false;
		state = _init_;
		break;
	}
}
