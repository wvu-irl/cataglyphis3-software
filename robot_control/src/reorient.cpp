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

#include <robot_control/reorient.h>

bool Reorient::runProc()
{
	switch(state)
	{
	case _init_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		procsToResume[procType] = false;
		clearSampleHistory();
		sendDriveRel(reorientDriveDistance, reorientPivotAngle, false, 0.0, false, false);
		sendDriveRel(0.0, -(90.0+reorientPivotAngle), false, 0.0, false, false);
		sendSearch(regionsOfInterestSrv.response.ROIList.at(currentROIIndex).whiteProb,
				   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).silverProb,
				   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).blueOrPurpleProb,
				   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).pinkProb,
				   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).redProb,
				   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).orangeProb,
				   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).yellowProb);
		reorientCount++;
		state = _exec_;
		resetQueueEmptyCondition();
		break;
	case _exec_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		procsToResume[procType] = false;
		computeDriveSpeeds();
		if(searchEnded() || queueEmptyTimedOut) state = _finish_;
		else state = _exec_;
		serviceQueueEmptyCondition();
		break;
	case _interrupt_:
		avoidLockout = false;
		performReorient = false;
		sampleDataActedUpon = true;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		procsToResume[procType] = false;
		state = _init_;
		break;
	case _finish_:
		avoidLockout = false;
		sampleDataActedUpon = true;
		performReorient = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		procsToResume[procType] = false;
		state = _init_;
		break;
	}
}
