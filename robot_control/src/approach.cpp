#include <robot_control/approach.h>

Approach::Approach()
{
	approachableSample = false;
	numSampleCandidates = 0;
    backUpCount = 0;
	commandedSearch = false;
}

bool Approach::runProc()
{
    //ROS_INFO("approachState = %i", state);
    //ROS_INFO("approachStep = %i", step);
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        grabberDistanceTolerance = initGrabberDistanceTolerance;
        grabberAngleTolerance = initGrabberAngleTolerance;
		findHighestConfSample();
		expectedSampleDistance = highestConfSample.distance;
		expectedSampleAngle = highestConfSample.bearing;
		//sampleTypeMux = (1 << (static_cast<uint8_t>(bestSample.type) & 255)) & 255;
		//ROS_INFO("approach sampleTypeMux = %u",sampleTypeMux);
		//sendSearch(sampleTypeMux);
        computeDriveSpeeds();
		step = _computeManeuver;
		state = _exec_;
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
		switch(step)
		{
		case _computeManeuver:
			sampleTypeMux = 0;
			computeSampleValuesWithExpectedDistance();
			if(bestSampleValue >= approachValueThreshold) approachableSample = true;
			else approachableSample = false;
			if(approachableSample)
			{
                if(sampleInPosition_())
				{
                    if(confirmCollectFailedCount>=confirmCollectsFailedBeforeSideGrab) sideOffsetGrab = true;
                    else sideOffsetGrab = false;
					sampleInCollectPosition = true;
					backUpCount = 0;
					state = _finish_;
				}
				else
				{
                    grabberDistanceTolerance += grabberDistanceToleranceIncrementPerApproachManeuver;
                    grabberAngleTolerance += grabberAngleToleranceIncrementPerApproachManeuver;
                    computeManeuver_();
					computeExpectedSampleLocation();
                    sendDriveRel(distanceToDrive, angleToTurn, false, 0.0, false, false);
                    sendSearch(252); // 252 = b11111100 -> cached = 1; purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
					commandedSearch = true;
					step = _performManeuver;
					state = _exec_;
                    voiceSay->call("saw sample. maneuvering");
                }
			}
			else
			{
				backUpCount++;
                if(backUpCount>=maxBackUpCount)
				{
					backUpCount = 0;
					state = _finish_;
                    voiceSay->call("too many back ups");
				}
				else
				{
					bestSample.distance = expectedSampleDistance;
					bestSample.bearing = expectedSampleAngle;
					distanceToDrive = backUpDistance;
					angleToTurn = 0.0;
					computeExpectedSampleLocation();
                    sendDriveRel(backUpDistance, 0.0, false, 0.0, false, false);
                    sendSearch(252); // 252 = b11111100 -> cached = 1; purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
					commandedSearch = true;
					step = _performManeuver;
					state = _exec_;
                    voiceSay->call("did not see sample. backing up");
				}
			}
			break;
		case _performManeuver:
			if(commandedSearch)
			{
                if(searchEnded())
				{
					findHighestConfSample();
					expectedSampleDistance = highestConfSample.distance;
					expectedSampleAngle = highestConfSample.bearing;
					step = _computeManeuver;
				}
				else step = _performManeuver;
			}
			else
			{
				if(execLastProcType == procType && execLastSerialNum == serialNum) step = _computeManeuver;
				else step = _performManeuver;
			}
			state = _exec_;
			break;
		}
		break;
	case _interrupt_:
		avoidLockout = false;
		backUpCount = 0;
		sampleDataActedUpon = true;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		step = _computeManeuver;
		state = _init_;
		break;
	case _finish_:
		avoidLockout = false;
		sampleDataActedUpon = true;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		step = _computeManeuver;
		state = _init_;
		break;
	}
}

void Approach::computeManeuver_()
{
    float deltaAngle; // degrees
    if(confirmCollectFailedCount>=confirmCollectsFailedBeforeSideGrab)
    {
        deltaAngle = RAD2DEG*asin(distanceToGrabber/bestSample.distance*sin(DEG2RAD*(180.0-sideGrabAngleOffset))); // degrees
        distanceToDrive = distanceToGrabber*sin(DEG2RAD*(sideGrabAngleOffset-deltaAngle))/sin(DEG2RAD*deltaAngle) + pitchCorrectionGain*robotStatus.pitchAngle;
        angleToTurn = bestSample.bearing - deltaAngle;
    }
    else
    {
        distanceToDrive = bestSample.distance - distanceToGrabber - blindDriveDistance + pitchCorrectionGain*robotStatus.pitchAngle;
        angleToTurn = bestSample.bearing;
    }
    if(distanceToDrive > maxDriveDistance) distanceToDrive = maxDriveDistance;
}

bool Approach::sampleInPosition_()
{
    if(confirmCollectFailedCount>=confirmCollectsFailedBeforeSideGrab)
    {
        return (bestSample.confidence >= definiteSampleConfThresh) && (fabs(bestSample.distance - distanceToGrabber) <= grabberDistanceTolerance) &&
                (fabs(bestSample.bearing - sideGrabAngleOffset) <= grabberAngleTolerance);
    }
    else
    {
        return (bestSample.confidence >= definiteSampleConfThresh) && (fabs(bestSample.distance - distanceToGrabber - blindDriveDistance) <= grabberDistanceTolerance) &&
                (fabs(bestSample.bearing) <= grabberAngleTolerance);
    }
}
