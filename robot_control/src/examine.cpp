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

#include <robot_control/examine.h>

bool Examine::runProc()
{
    //ROS_INFO("examineState = %i",state);
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
            if(!timers[_roiTimer_]->running && !roiTimeExpired) {timers[_roiTimer_]->setPeriod(allocatedROITime); timers[_roiTimer_]->start();}
            computeDriveSpeeds();
            examineCount++;
            if(examineCount>examineLimit) {possibleSample = false; definiteSample = false; examineCount = 0; state = _finish_; break;}
            //findHighestConfSample();
            expectedSampleDistance = highestConfSample.distance;
            expectedSampleAngle = highestConfSample.bearing;
            if(expectedSampleDistance < (examineOffsetDistance/2.0)) expectedSampleDistance = (examineOffsetDistance/2.0);
            examineOffsetAngle = RAD2DEG*2.0*asin(examineOffsetDistance/(2.0*expectedSampleDistance));
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
            sendSearch(regionsOfInterestSrv.response.ROIList.at(currentROIIndex).whiteProb,
                       regionsOfInterestSrv.response.ROIList.at(currentROIIndex).silverProb,
                       regionsOfInterestSrv.response.ROIList.at(currentROIIndex).blueOrPurpleProb,
                       regionsOfInterestSrv.response.ROIList.at(currentROIIndex).pinkProb,
                       regionsOfInterestSrv.response.ROIList.at(currentROIIndex).redProb,
                       regionsOfInterestSrv.response.ROIList.at(currentROIIndex).orangeProb,
                       regionsOfInterestSrv.response.ROIList.at(currentROIIndex).yellowProb);
            state = _exec_;
        }
        resetQueueEmptyCondition();
		break;
	case _exec_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        if(roiTimeExpired && !roiOvertimeExpired && !timers[_roiOvertimeTimer_]->running) {timers[_roiOvertimeTimer_]->start(); ROS_INFO("roi overtime started");}
		computeDriveSpeeds();
        if(searchEnded() || queueEmptyTimedOut) state = _finish_;
		else state = _exec_;
        serviceQueueEmptyCondition();
		break;
	case _interrupt_:
        avoidLockout = true;
		examineCount = 0;
		sampleDataActedUpon = true;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
        procsToResume[procType] = false;
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
