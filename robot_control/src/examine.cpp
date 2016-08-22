#include <robot_control/examine.h>

bool Examine::runProc()
{
    //ROS_INFO("examineState = %i",state);
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		computeDriveSpeeds();
		examineCount++;
		if(examineCount>examineLimit) {possibleSample = false; definiteSample = false; examineCount = 0; state = _finish_; break;}
		findHighestConfSample();
		expectedSampleDistance = highestConfSample.distance;
		expectedSampleAngle = highestConfSample.bearing;
		//distanceToDrive = sqrt(pow(offsetPositionDistance,2.0)+pow(expectedSampleDistance,2.0)-2.0*offsetPositionDistance*expectedSampleDistance*cos(DEG2RAD*offsetPositionAngle));
        if(expectedSampleDistance > maxDistanceToDrive) distanceToDrive = maxDistanceToDrive;
        else distanceToDrive = expectedSampleDistance;
		//angleToTurn = fabs(expectedSampleAngle)-RAD2DEG*asin(offsetPositionDistance/distanceToDrive*sin(DEG2RAD*offsetPositionAngle));
        if(expectedSampleAngle >= 0.0)
        {
            angleToTurn = expectedSampleAngle - examineOffsetAngle;
            finalAngleToTurn = 90.0 - examineOffsetAngle/2.0;
        }
        else
        {
            angleToTurn = expectedSampleAngle + examineOffsetAngle;
            finalAngleToTurn = -90.0 + examineOffsetAngle/2.0;
        }
        sendDriveRel(distanceToDrive, angleToTurn, false, 0.0, false, false);
        sendDriveRel(0.0, finalAngleToTurn, false, 0.0, false, false);
		sendSearch(252); // 252 = b11111100 -> cached = 1; purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
		state = _exec_;
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		computeDriveSpeeds();
        if(searchEnded()) state = _finish_;
		else state = _exec_;
		break;
	case _interrupt_:
		avoidLockout = false;
		examineCount = 0;
		sampleDataActedUpon = true;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _init_;
		break;
	case _finish_:
		avoidLockout = false;
		sampleDataActedUpon = true;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		state = _init_;
		break;
	}
}
