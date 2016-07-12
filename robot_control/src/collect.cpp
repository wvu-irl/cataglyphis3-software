#include <robot_control/collect.h>

bool Collect::runProc()
{
	ROS_INFO("collectState = %i", state);
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        sendDriveRel(blindDriveDistance, 0.0, false, 0.0, false);
		sendGrab();
        computeDriveSpeeds();
		state = _exec_;
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        computeDriveSpeeds();
		if(execLastProcType == procType && execLastSerialNum == serialNum) state = _finish_;
		else state = _exec_;
		break;
	case _interrupt_:
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _init_;
		break;
	case _finish_:
		avoidLockout = false;
		possessingSample = true;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		state = _init_;
		break;
	}
}
