#include <robot_control/go_home.h>

bool GoHome::runProc()
{
	ROS_INFO("goHomeState = %i", state);
	switch(state)
	{
	case _init_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        computeDriveSpeeds();
        avoidCount = 0;
        prevAvoidCountDecXPos = robotStatus.xPos;
        prevAvoidCountDecYPos = robotStatus.yPos;
        examineCount = 0;
        confirmCollectFailedCount = 0;
		numWaypointsToTravel = 1;
		clearAndResizeWTT();
		waypointsToTravel.at(0).x = 5.0;
		waypointsToTravel.at(0).y = 0.0;
		callIntermediateWaypoints();
        sendDriveGlobal(false);
		state = _exec_;
		break;
	case _exec_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        computeDriveSpeeds();
        serviceAvoidCounterDecrement();
		if(execLastProcType == procType && execLastSerialNum == serialNum) state = _finish_;
		else state = _exec_;
		break;
	case _interrupt_:
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _exec_;
		break;
	case _finish_:
		avoidLockout = false;
		atHome = true;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		state = _init_;
		break;
	}
}
