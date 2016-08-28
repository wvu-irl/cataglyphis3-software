#include <robot_control/go_home.h>

bool GoHome::runProc()
{
    //ROS_INFO("goHomeState = %i", state);
	switch(state)
	{
	case _init_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        avoidCount = 0;
        prevAvoidCountDecXPos = robotStatus.xPos;
        prevAvoidCountDecYPos = robotStatus.yPos;
        examineCount = 0;
        confirmCollectFailedCount = 0;
		numWaypointsToTravel = 1;
		clearAndResizeWTT();
        waypointsToTravel.at(0).x = homeWaypointX;
        waypointsToTravel.at(0).y = homeWaypointY;
        waypointsToTravel.at(0).unskippable = false;
        waypointsToTravel.at(0).maxAvoids = maxHomeWaypointAvoidCount;
		callIntermediateWaypoints();
        sendDriveAndWait(lidarUpdateWaitTime, true, 180.0);
		state = _exec_;
		break;
	case _exec_:
        if(execLastProcType == procType && execLastSerialNum == (serialNum-1)) avoidLockout = true; // *** Not sure if this is really the right condition...
        else avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
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
        if(robotStatus.homingUpdated)
        {
            //timers[_homingTimer_]->stop();
            //timers[_homingTimer_]->start();
            homingUpdatedFailedCount = 0;
            performHoming = false;
            homingUpdateFailed = false;
            useDeadReckoning = false;
            if(hypot(homeWaypointX - robotStatus.xPos, homeWaypointY - robotStatus.yPos) > goHomeDistanceTolerance) atHome = false;
            else atHome = true;
        }
        else
        {
            homingUpdatedFailedCount++;
            homingUpdateFailed = true;
            atHome = false;
        }
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		state = _init_;
		break;
	}
}
