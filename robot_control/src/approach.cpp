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
		//sendSearch(sampleTypeMux);
		sendSearch(124); // 124 = b1111100 -> purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
		state = _exec_;
		break;
	case _exec_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		switch(step)
		{
		case _performSearch:
			ROS_INFO("procType = %i",static_cast<int>(this->procType));
			ROS_INFO("serialNum = %i",this->serialNum);
			ROS_INFO("cvSamplesFoundMsg.procType = %i",static_cast<int>(cvSamplesFoundMsg.procType));
			ROS_INFO("cvSamplesFoundMsg.serialNum = %i",cvSamplesFoundMsg.serialNum);
			if(cvSamplesFoundMsg.procType==this->procType && cvSamplesFoundMsg.serialNum==this->serialNum)
			{
				if((bestSample.confidence >= definiteSampleConfThresh) && (fabs(bestSample.distance - distanceToGrabber - blindDriveDistance) <= grabberDistanceTolerance) &&
						(fabs(bestSample.bearing) <= grabberAngleTolerance))
				{
					sendDriveRel(blindDriveDistance, 0.0, false, 0.0, false, false);
					step = _blindDrive;
					state = _exec_;
				}
				else if(bestSample.confidence==0)
				{
					backUpCount++;
					if(backUpCount>maxBackUpCount)
					{
						backUpCount = 0;
						state = _finish_;
					}
					else
					{
						sendDriveRel(backUpDistance, 0.0, false, 0.0, false, false);
						step = _driveManeuver;
						state = _exec_;
					}
				}
				else
				{
					distanceToDrive = bestSample.distance - distanceToGrabber - blindDriveDistance;
					if(distanceToDrive > maxDriveDistance) distanceToDrive = maxDriveDistance;
					angleToTurn = bestSample.bearing;
					sendDriveRel(distanceToDrive, angleToTurn, false, 0.0, false, false);
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
				backUpCount = 0;
				sampleInCollectPosition = true;
				state = _finish_;
			}
			else state = _exec_;
			break;
		}
		break;
	case _interrupt_:
		backUpCount = 0;
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
