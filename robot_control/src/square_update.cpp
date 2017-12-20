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

#include <robot_control/square_update.h>

bool SquareUpdate::runProc()
{
	//ROS_INFO("squareUpdateState = %i",state);
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		computeDriveSpeeds();
		avoidCount = 0;
        numWaypointsToTravel = 1;
		clearAndResizeWTT();
		waypointsToTravel.at(0).x = cornerX;
		waypointsToTravel.at(0).y = cornerY;
        waypointsToTravel.at(0).maxAvoids = maxCornerWaypointAvoidCount;
        sendDriveAndWait(lidarUpdateWaitTime, true, 180.0);
        clearAndResizeWTT();
        waypointsToTravel.at(0).x = -cornerX;
        waypointsToTravel.at(0).y = cornerY;
        waypointsToTravel.at(0).maxAvoids = maxCornerWaypointAvoidCount;
        sendDriveAndWait(lidarUpdateWaitTime, true, -90.0);
        clearAndResizeWTT();
        waypointsToTravel.at(0).x = -cornerX;
        waypointsToTravel.at(0).y = -cornerY;
        waypointsToTravel.at(0).maxAvoids = maxCornerWaypointAvoidCount;
        sendDriveAndWait(lidarUpdateWaitTime, true, 0.0);
        clearAndResizeWTT();
        waypointsToTravel.at(0).x = cornerX;
        waypointsToTravel.at(0).y = -cornerY;
        waypointsToTravel.at(0).maxAvoids = maxCornerWaypointAvoidCount;
        sendDriveAndWait(lidarUpdateWaitTime, true, 90.0);
        clearAndResizeWTT();
        waypointsToTravel.at(0).x = homeWaypointX;
        waypointsToTravel.at(0).y = homeWaypointY;
        waypointsToTravel.at(0).maxAvoids = maxCornerWaypointAvoidCount;
		sendDriveAndWait(lidarUpdateWaitTime, true, 180.0);
		state = _exec_;
        resetQueueEmptyCondition();
		break;
	case _exec_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		computeDriveSpeeds();
        if((execLastProcType == procType && execLastSerialNum == serialNum) || queueEmptyTimedOut) state = _finish_;
		else state = _exec_;
        serviceQueueEmptyCondition();
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
			timers[_homingTimer_]->stop();
			timers[_homingTimer_]->start();
			performHoming = false;
			homingUpdatedFailedCount = 0;
			homingUpdateFailed = false;
			useDeadReckoning = false;
		}
		else
		{
			homingUpdatedFailedCount++;
			homingUpdateFailed = true;
			if(homingUpdatedFailedCount>=homingFailedSwitchToDeadReckoningCount)
			{
				useDeadReckoning = true; // !!! This switch in hsm probably won't happen quickly enough before the next cycle of planning a manuever. How to handle this?
			}
			if(homingUpdatedFailedCount>=maxHomingUpdatedFailedCount)
			{
				performSafeMode = true;
			}
		}
		atHome = true;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		state = _init_;
		break;
	}
}
