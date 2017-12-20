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

#include <robot_control/initialize.h>

bool Initialize::runProc()
{
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		procsToResume[procType] = false;
		sendWait(initWaitTime, false);
		sendOpen(); // Make sure the grabber is open initially
		sendDriveRel(driveOffPlatformDistance, 0.0, false, 0.0, false, false);
		step = _drivingOffPlatform;
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
		case _drivingOffPlatform:
			if((execLastProcType == procType && execLastSerialNum == serialNum) || queueEmptyTimedOut)
			{
				biasRemovalTimedOut = false;
				timers[_biasRemovalActionTimeoutTimer_]->start();
				navControlSrv.request.runBiasRemoval = true;
				if(navControlClient.call(navControlSrv)) ROS_DEBUG("navFilterControlService call successful");
				else ROS_ERROR("navFilterControlService call unsuccessful");
				step = _firstBiasRemoval;
			}
			else
			{
				step = _drivingOffPlatform;
				state = _exec_;
			}
			break;
		case _firstBiasRemoval:
			if(navStatus!=0 || biasRemovalTimedOut)
			{
				timers[_biasRemovalActionTimeoutTimer_]->stop();
				timers[_biasRemovalTimer_]->stop();
				timers[_biasRemovalTimer_]->start();
				biasRemovalTimedOut = false;
				performBiasRemoval = false;
				state = _finish_;
			}
			else
			{
				step = _firstBiasRemoval;
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
		state = _finish_;
		break;
	case _finish_:
		startSLAM = true;
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		procsToResume[procType] = false;
		initialized = true;
		state = _finish_;
		break;
	}
}
