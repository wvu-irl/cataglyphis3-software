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

#include <robot_control/deposit_sample.h>

bool DepositSample::runProc()
{
    //ROS_INFO("depositSampleState = %i", state);
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        dropFailed = false;
        driveSpeedsMsg.vMax = slowVMax;
        driveSpeedsMsg.rMax = defaultRMax;
        driveSpeedsMsgPrev.vMax = slowVMax;
        driveSpeedsMsgPrev.rMax = defaultRMax;
        driveSpeedsPub.publish(driveSpeedsMsg);
		samplesCollected++;
		if(samplesCollected<MAX_SAMPLES)
		{
			sendDrop();
            sendDriveRel(-3.0, 0.0, false, 0.0, false, false);
			missionEnded = false;
			state = _exec_;
            voiceSay->call("I'll be back");
		}
		else
		{
			missionEnded = true;
			state = _finish_;
            voiceSay->call("mission ended");
		}
        resetQueueEmptyCondition();
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        if((execLastProcType == procType && execLastSerialNum == serialNum) || queueEmptyTimedOut) state = _finish_;
		else state = _exec_;
        if(grabberStatusMsg.dropFailed) {dropFailed = true; ROS_WARN("deposit sample drop failed");}
        else dropFailed = false;
        serviceQueueEmptyCondition();
		break;
	case _interrupt_:
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _init_;
		break;
	case _finish_:
        if(dropFailed)
        {
            avoidLockout = true;
            possessingSample = true;
            confirmedPossession = true;
            atHome = true;
            inDepositPosition = false;
            //startSLAM = false;
        }
        else
        {
            avoidLockout = false;
            possessingSample = false;
            confirmedPossession = false;
            atHome = false;
            inDepositPosition = false;
            possibleSample = false;
            definiteSample = false;
            sampleDataActedUpon = false;
            sampleInCollectPosition = false;
            //startSLAM = true;
        }
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		state = _init_;
		break;
	}
}
