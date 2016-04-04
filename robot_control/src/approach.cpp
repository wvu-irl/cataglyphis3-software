#include <robot_control/approach.h>

Approach::Approach()
{
	approachableSample = false;
	numSampleCandidates = 0;
	commandedSearch = false;
}

bool Approach::runProc()
{
	ROS_INFO("approachState = %i", state);
	ROS_INFO("approachStep = %i", step);
	switch(state)
	{
	case _init_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		findHighestConfSample();
		expectedSampleDistance = highestConfSample.distance;
		expectedSampleAngle = highestConfSample.bearing;
		//sampleTypeMux = (1 << (static_cast<uint8_t>(bestSample.type) & 255)) & 255;
		//ROS_INFO("approach sampleTypeMux = %u",sampleTypeMux);
		//sendSearch(sampleTypeMux);
		step = _computeManeuver;
		state = _exec_;
		break;
	case _exec_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		switch(step)
		{
		case _computeManeuver:
			sampleTypeMux = 0;
			computeSampleValues();
			if(approachableSample)
			{
				if((bestSample.confidence >= definiteSampleConfThresh) && (fabs(bestSample.distance - distanceToGrabber - blindDriveDistance) <= grabberDistanceTolerance) &&
						(fabs(bestSample.bearing) <= grabberAngleTolerance))
				{
					sampleInCollectPosition = true;
					backUpCount = 0;
					state = _finish_;
				}
				else
				{
					distanceToDrive = bestSample.distance - distanceToGrabber - blindDriveDistance;
					if(distanceToDrive > maxDriveDistance) distanceToDrive = maxDriveDistance;
					angleToTurn = bestSample.bearing;
					computeExpectedSampleLocation();
					sendDriveRel(distanceToDrive, angleToTurn, false, 0.0, false, false);
					sendSearch(124); // 124 = b1111100 -> purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
					commandedSearch = true;
					step = _performManeuver;
					state = _exec_;
				}
			}
			else
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
					commandedSearch = false;
					step = _performManeuver;
					state = _exec_;
				}
			}
			break;
		case _performManeuver:
			if(commandedSearch)
			{
				if(cvSamplesFoundMsg.procType==this->procType && cvSamplesFoundMsg.serialNum==this->serialNum) step = _computeManeuver;
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
		backUpCount = 0;
		sampleDataActedUpon = true;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		step = _computeManeuver;
		state = _init_;
		break;
	case _finish_:
		sampleDataActedUpon = true;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		step = _computeManeuver;
		state = _init_;
		break;
	}
}

void Approach::computeSampleValues()
{
	numSampleCandidates = cvSamplesFoundMsg.sampleList.size();
	sampleValues.clear();
	sampleValues.resize(numSampleCandidates);
	bestSampleValue = 0;
	approachableSample = false;
	for(int i=0; i<numSampleCandidates; i++)
	{
		sampleValues.at(i) = (sampleConfidenceGain*cvSamplesFoundMsg.sampleList.at(i).confidence -
								(int)(sampleDistanceToExpectedGain*sqrt(pow(cvSamplesFoundMsg.sampleList.at(i).distance,2)+pow(expectedSampleDistance,2)-
									2*cvSamplesFoundMsg.sampleList.at(i).distance*expectedSampleDistance*
										cos(DEG2RAD*(cvSamplesFoundMsg.sampleList.at(i).bearing-expectedSampleAngle)))))/sampleConfidenceGain;

		if(sampleValues.at(i) > bestSampleValue) {bestSample = cvSamplesFoundMsg.sampleList.at(i); bestSampleValue = sampleValues.at(i);}
		if(bestSampleValue >= approachValueThreshold) approachableSample = true;
	}
}

void Approach::computeExpectedSampleLocation()
{
	expectedSampleDistance = pow(bestSample.distance,2) + pow(distanceToDrive,2) - 2*bestSample.distance*distanceToDrive*cos(DEG2RAD*(bestSample.bearing));
	if(distanceToDrive < bestSample.distance) expectedSampleAngle = bestSample.bearing + RAD2DEG*asin(DEG2RAD*(distanceToDrive/expectedSampleDistance*sin(DEG2RAD*bestSample.bearing)));
	else
	{
		if(bestSample.bearing >= 0.0) expectedSampleAngle = 180.0 - RAD2DEG*asin(bestSample.distance/expectedSampleDistance*sin(DEG2RAD*bestSample.bearing));
		else expectedSampleAngle = -180.0 - RAD2DEG*asin(bestSample.distance/expectedSampleDistance*sin(DEG2RAD*bestSample.bearing));
	}
}

void Approach::findHighestConfSample()
{
	highestConfSample.confidence = 0;
	for(int i=0; i<cvSamplesFoundMsg.sampleList.size(); i++)
	{
		if(cvSamplesFoundMsg.sampleList.at(i).confidence > highestConfSample.confidence) highestConfSample = cvSamplesFoundMsg.sampleList.at(i);
	}
}
