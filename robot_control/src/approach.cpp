/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        if(roiOvertimeExpired)
        {
            roiOvertimeExpired = false;
            possibleSample = false;
            definiteSample = false;
            state = _finish_;
        }
        else
        {
            if(!timers[_roiTimer_]->running && !roiTimeExpired) {roiTimeExpired = false; timers[_roiTimer_]->setPeriod(allocatedROITime); timers[_roiTimer_]->start();}
            grabberDistanceTolerance = initGrabberDistanceTolerance;
            grabberAngleTolerance = initGrabberAngleTolerance;
            //findHighestConfSample();
            expectedSampleDistance = highestConfSample.distance;
            expectedSampleAngle = highestConfSample.bearing;
            //sampleTypeMux = (1 << (static_cast<uint8_t>(highestConfSample.type) & 255)) & 255;
            //ROS_INFO("approach sampleTypeMux = %u",sampleTypeMux);
            //sendSearch(sampleTypeMux);
            computeDriveSpeeds();
            step = _computeManeuver;
            state = _exec_;
        }
        resetQueueEmptyCondition();
		break;
	case _exec_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
		switch(step)
		{
		case _computeManeuver:
            resetQueueEmptyCondition();
			sampleTypeMux = 0;
			computeSampleValuesWithExpectedDistance(true);
            if(sampleDistanceAdjustedConf >= approachValueThreshold && definiteSample) approachableSample = true;
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
                else if(roiOvertimeExpired)
                {
                    backUpCount = 0;
                    roiOvertimeExpired = false;
                    possibleSample = false;
                    definiteSample = false;
                    state = _finish_;
                }
				else
				{
                    grabberDistanceTolerance += grabberDistanceToleranceIncrementPerApproachManeuver;
                    grabberAngleTolerance += grabberAngleToleranceIncrementPerApproachManeuver;
                    computeManeuver_();
					computeExpectedSampleLocation();
                    sendDriveRel(distanceToDrive, angleToTurn, false, 0.0, false, false);
                    sendSearch(regionsOfInterestSrv.response.ROIList.at(currentROIIndex).whiteProb,
                               regionsOfInterestSrv.response.ROIList.at(currentROIIndex).silverProb,
                               regionsOfInterestSrv.response.ROIList.at(currentROIIndex).blueOrPurpleProb,
                               regionsOfInterestSrv.response.ROIList.at(currentROIIndex).pinkProb,
                               regionsOfInterestSrv.response.ROIList.at(currentROIIndex).redProb,
                               regionsOfInterestSrv.response.ROIList.at(currentROIIndex).orangeProb,
                               regionsOfInterestSrv.response.ROIList.at(currentROIIndex).yellowProb);
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
                    highestConfSample.distance = expectedSampleDistance;
                    highestConfSample.bearing = expectedSampleAngle;
					distanceToDrive = backUpDistance;
					angleToTurn = 0.0;
					computeExpectedSampleLocation();
                    sendDriveRel(backUpDistance, 0.0, false, 0.0, false, false);
                    sendSearch(regionsOfInterestSrv.response.ROIList.at(currentROIIndex).whiteProb,
                               regionsOfInterestSrv.response.ROIList.at(currentROIIndex).silverProb,
                               regionsOfInterestSrv.response.ROIList.at(currentROIIndex).blueOrPurpleProb,
                               regionsOfInterestSrv.response.ROIList.at(currentROIIndex).pinkProb,
                               regionsOfInterestSrv.response.ROIList.at(currentROIIndex).redProb,
                               regionsOfInterestSrv.response.ROIList.at(currentROIIndex).orangeProb,
                               regionsOfInterestSrv.response.ROIList.at(currentROIIndex).yellowProb);
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
                if(searchEnded() || queueEmptyTimedOut)
				{
                    //findHighestConfSample();
					expectedSampleDistance = highestConfSample.distance;
					expectedSampleAngle = highestConfSample.bearing;
					step = _computeManeuver;
				}
				else step = _performManeuver;
			}
			else
			{
                if((execLastProcType == procType && execLastSerialNum == serialNum) || queueEmptyTimedOut) step = _computeManeuver;
				else step = _performManeuver;
			}
			state = _exec_;
			break;
		}
        if(roiTimeExpired && !roiOvertimeExpired && !timers[_roiOvertimeTimer_]->running) {timers[_roiOvertimeTimer_]->start(); ROS_INFO("roi overtime started");}
        serviceQueueEmptyCondition();
		break;
	case _interrupt_:
        avoidLockout = true;
		backUpCount = 0;
		sampleDataActedUpon = true;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
        procsToResume[procType] = false;
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
        deltaAngle = RAD2DEG*asin(distanceToGrabber/highestConfSample.distance*sin(DEG2RAD*(180.0-sideGrabAngleOffset))); // degrees
        distanceToDrive = distanceToGrabber*sin(DEG2RAD*(sideGrabAngleOffset-deltaAngle))/sin(DEG2RAD*deltaAngle) + pitchCorrectionGain*robotStatus.pitchAngle;
        angleToTurn = highestConfSample.bearing - deltaAngle;
    }
    else
    {
        distanceToDrive = highestConfSample.distance - distanceToBlindDriveLocation - blindDriveDistance + pitchCorrectionGain*robotStatus.pitchAngle;
        angleToTurn = highestConfSample.bearing;
    }
    if(distanceToDrive > maxDriveDistance) distanceToDrive = maxDriveDistance;
}

bool Approach::sampleInPosition_()
{
    if(confirmCollectFailedCount>=confirmCollectsFailedBeforeSideGrab)
    {
        return (sampleDistanceAdjustedConf >= definiteSampleConfThresh) && (fabs(highestConfSample.distance - distanceToGrabber) <= grabberDistanceTolerance) &&
                (fabs(highestConfSample.bearing - sideGrabAngleOffset) <= grabberAngleTolerance);
    }
    else
    {
        return (sampleDistanceAdjustedConf >= definiteSampleConfThresh) && (fabs(highestConfSample.distance - distanceToGrabber - blindDriveDistance) <= grabberDistanceTolerance) &&
                (fabs(highestConfSample.bearing) <= grabberAngleTolerance);
    }
}
