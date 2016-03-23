#include <robot_control/collect.h>

bool Collect::runProc()
{
	ROS_INFO("collectState = %i", state);
	switch(state)
	{
	case _init_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		sendGrab();
		sendDriveRel(-1.0, 0.0, false, 0.0, false);
		state = _exec_;
		break;
	case _exec_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		if(execLastProcType == procType && execLastSerialNum == serialNum) state = _finish_;
		else state = _exec_;
		break;
	case _interrupt_:
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _init_;
		break;
	case _finish_:
		possessingSample = true;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		state = _init_;
		break;
	}
}
