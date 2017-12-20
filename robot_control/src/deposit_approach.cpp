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

#include <robot_control/deposit_approach.h>

DepositApproach::DepositApproach()
{
	depositLocations.resize(MAX_SAMPLES);
	depositLocations.at(0).x = 0.8;
	depositLocations.at(0).y = 0.0;

	depositLocations.at(1).x = 0.8;
    depositLocations.at(1).y = -0.55;

    depositLocations.at(2).x = 1.3;
    depositLocations.at(2).y = -0.55;

	depositLocations.at(3).x = 0.8;
    depositLocations.at(3).y = 0.55;

    depositLocations.at(4).x = 1.3;
	depositLocations.at(4).y = 0.0;

    depositLocations.at(5).x = 1.8;
    depositLocations.at(5).y = -0.55;

    depositLocations.at(6).x = 1.3;
    depositLocations.at(6).y = 0.55;

    depositLocations.at(7).x = 1.8;
    depositLocations.at(7).y = 0.0;

    depositLocations.at(8).x = 1.8;
    depositLocations.at(8).y = 0.55;

    depositLocations.at(9).x = 1.75;
	depositLocations.at(9).y = 0.0;
}

bool DepositApproach::runProc()
{
    //ROS_INFO("depositApproachState = %i", state);
    //ROS_INFO("depositApproachStep = %i",step);
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		//startSLAM = false;
		procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        driveSpeedsMsg.vMax = slowVMax;
        driveSpeedsMsg.rMax = defaultRMax;
        driveSpeedsMsgPrev.vMax = slowVMax;
        driveSpeedsMsgPrev.rMax = defaultRMax;
        driveSpeedsPub.publish(driveSpeedsMsg);
		numWaypointsToTravel = 1;
		clearAndResizeWTT();
		waypointsToTravel.at(0).x = depositAlignXDistance;
		waypointsToTravel.at(0).y = depositLocations.at(samplesCollected).y;
        waypointsToTravel.at(0).maxAvoids = maxHomeWaypointAvoidCount;
		callIntermediateWaypoints();
        if(!tiltTooExtremeForBiasRemoval)
        {
            biasRemovalTimedOut = false;
            timers[_biasRemovalActionTimeoutTimer_]->start();
            navControlSrv.request.runBiasRemoval = true;
            if(navControlClient.call(navControlSrv)) ROS_DEBUG("navFilterControlService call successful");
            else ROS_ERROR("navFilterControlService call unsuccessful");
            step = _biasRemoval;
        }
        else
        {
            voiceSay->call("Tilt too extreme for bias removal. Moving on.");
            step = _align;
        }
		state = _exec_;
        resetQueueEmptyCondition();
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
		switch(step)
		{
        case _biasRemoval:
            if(navStatus!=0 || biasRemovalTimedOut)
            {
                timers[_biasRemovalActionTimeoutTimer_]->stop();
                timers[_biasRemovalTimer_]->stop();
                timers[_biasRemovalTimer_]->start();
                biasRemovalTimedOut = false;
                performBiasRemoval = false;
                sendDriveGlobal(false, false, 0.0);
                sendWait(lidarUpdateWaitTime, false);
                step = _align;
            }
            else step = _biasRemoval;
            state = _exec_;
            break;
		case _align:
			if(execLastProcType == procType && execLastSerialNum == serialNum)
			{
				calcPlatformDrive();
                sendDriveRel(platformDriveDistance, platformPivotAngle, false, 0.0, false, false);
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
            if((execLastProcType == procType && execLastSerialNum == serialNum) || queueEmptyTimedOut)
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
        serviceQueueEmptyCondition();
		break;
	case _interrupt_:
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
        step = _biasRemoval;
		state = _exec_;
		break;
	case _finish_:
		avoidLockout = true;
		inDepositPosition = true;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        step = _biasRemoval;
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
