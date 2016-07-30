#include <robot_control/deposit_approach.h>

DepositApproach::DepositApproach()
{
	depositLocations.resize(MAX_SAMPLES);
	depositLocations.at(0).x = 0.8;
	depositLocations.at(0).y = 0.0;

	depositLocations.at(1).x = 0.8;
	depositLocations.at(1).y = -0.483;

	depositLocations.at(2).x = 1.25;
	depositLocations.at(2).y = -0.483;

	depositLocations.at(3).x = 0.8;
	depositLocations.at(3).y = 0.483;

	depositLocations.at(4).x = 1.25;
	depositLocations.at(4).y = 0.0;

	depositLocations.at(5).x = 1.7;
	depositLocations.at(5).y = -0.483;

	depositLocations.at(6).x = 1.25;
	depositLocations.at(6).y = 0.483;

	depositLocations.at(7).x = 1.7;
	depositLocations.at(7).y = 0.0;

	depositLocations.at(8).x = 1.7;
	depositLocations.at(8).y = 0.483;

	depositLocations.at(9).x = 1.7;
	depositLocations.at(9).y = 0.0;
}

bool DepositApproach::runProc()
{
	ROS_INFO("depositApproachState = %i", state);
	ROS_INFO("depositApproachStep = %i",step);
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		step = _align;
		procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        driveSpeedsMsg.vMax = slowVMax;
        driveSpeedsMsg.rMax = defaultRMax;
        driveSpeedsMsgPrev.vMax = slowVMax;
        driveSpeedsMsgPrev.rMax = defaultRMax;
        driveSpeedsPub.publish(driveSpeedsMsg);
		numWaypointsToTravel = 1;
		clearAndResizeWTT();
		waypointsToTravel.at(0).x = depositAlignXDistance;
		waypointsToTravel.at(0).y = depositLocations.at(samplesCollected).y;
		callIntermediateWaypoints();
        sendDriveGlobal(false, false, 0.0);
		state = _exec_;
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
		switch(step)
		{
		case _align:
			if(execLastProcType == procType && execLastSerialNum == serialNum)
			{
				calcPlatformDrive();
                sendDriveRel(platformDriveDistance, platformPivotAngle, false, 0.0, false);
				step = _platform;
				state = _exec_;
			}
			else
			{
				step = _align;
				state = _exec_;
			}
			break;
		case _platform:
			if(execLastProcType == procType && execLastSerialNum == serialNum)
			{
				state = _finish_;
			}
			else
			{
				step = _platform;
				state = _exec_;
			}
			break;
		}
		break;
	case _interrupt_:
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		step = _align;
		state = _exec_;
		break;
	case _finish_:
		avoidLockout = false;
		inDepositPosition = true;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		step = _align;
		state = _init_;
		break;
	}
}

void DepositApproach::calcPlatformDrive()
{
	platformDriveDistance = hypot((robotStatus.xPos - depositLocations.at(samplesCollected).x), (robotStatus.yPos - depositLocations.at(samplesCollected).y)) - distanceToGrabber;
	if(robotStatus.heading>=0.0) platformPivotAngle = 180.0 - fmod(robotStatus.heading,360.0) + RAD2DEG*atan2((robotStatus.yPos - depositLocations.at(samplesCollected).y), (robotStatus.xPos - depositLocations.at(samplesCollected).x));
	else platformPivotAngle = -180.0 - fmod(robotStatus.heading,360.0) + RAD2DEG*atan2((robotStatus.yPos - depositLocations.at(samplesCollected).y), (robotStatus.xPos - depositLocations.at(samplesCollected).x));
}
