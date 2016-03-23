#include <robot_control/approach.h>

bool Approach::runProc()
{
	ROS_INFO("approachState = %i", state);
	ROS_INFO("approachStep = %i", step);
	switch(state)
	{
	case _init_:
		step = _performSearch;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		sampleTypeMux = 0;
		sampleTypeMux = (1 << (static_cast<uint8_t>(bestSample.type) & 255)) & 255;
		ROS_INFO("approach sampleTypeMux = %u",sampleTypeMux);
		sendSearch(sampleTypeMux);
		state = _exec_;
		break;
	case _exec_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		switch(step)
		{
		case _performSearch:
			if(cvSamplesFoundMsg.procType==this->procType && cvSamplesFoundMsg.serialNum==this->serialNum &&
					execLastProcType == procType && execLastSerialNum == serialNum)
			{
				if((bestSample.confidence >= definiteSampleConfThresh) && (fabs(bestSample.distance - distanceToGrabber) <= grabberDistanceTolerance) &&
						(fabs(bestSample.bearing) <= grabberAngleTolerance))
				{
					sendDriveRel(blindDriveDistance, 0.0, false, 0.0, false);
					step = _blindDrive;
					state = _exec_;
				}
				else
				{
					distanceToDrive = bestSample.distance - distanceToGrabber;
					angleToTurn = bestSample.bearing;
					sendDriveRel(distanceToDrive, angleToTurn, false, 0.0, false);
					step = _driveManeuver;
					state = _exec_;
				}
			}
			else
			{
				step = _performSearch;
				state = _exec_;
			}
			break;
		case _driveManeuver:
			if(execLastProcType == procType && execLastSerialNum == serialNum) state = _finish_;
			else state = _exec_;
			break;
		case _blindDrive:
			if(execLastProcType == procType && execLastSerialNum == serialNum)
			{
				sampleInCollectPosition = true;
				state = _finish_;
			}
			else state = _exec_;
			break;
		}
		break;
	case _interrupt_:
		sampleDataActedUpon = true;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _init_;
		step = _performSearch;
		break;
	case _finish_:
		sampleDataActedUpon = true;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		state = _init_;
		step = _performSearch;
		break;
	}
}
