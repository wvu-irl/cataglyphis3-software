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

#include <robot_control/drive_relative.h>

void DriveRelative::init()
{
    dropFailed_ = false;
    slidesFailed_ = false;
	distanceToDrive_ = params.float1;
	angleToTurn_ = params.float2;
    desiredEndHeading_ = params.float3;
	endHeading_ = params.bool1;
    pushedToFront_ = params.bool2;
    closedLoop_ = params.bool3;
    nextGlobalX = distanceToDrive_*cos(DEG2RAD*(angleToTurn_ + robotStatus.heading)) + robotStatus.xPos;
    nextGlobalY = distanceToDrive_*sin(DEG2RAD*(angleToTurn_ + robotStatus.heading)) + robotStatus.yPos;
    clearDeques();
	pushTask(_pivot_);
	driveDeque.back()->params.float1 = angleToTurn_;
    if(closedLoop_) pushTask(_driveStraightCL_);
    else pushTask(_driveStraight_);
	driveDeque.back()->params.float1 = distanceToDrive_;
	if(endHeading_)
	{
		pushTask(_pivot_);
        candidateEndHeadingAngleToTurn_[0] = desiredEndHeading_ - (fmod(robotStatus.heading, 360.0) + angleToTurn_);
        candidateEndHeadingAngleToTurn_[1] = -desiredEndHeading_ - (fmod(robotStatus.heading, 360.0) + angleToTurn_);
        if(fabs(candidateEndHeadingAngleToTurn_[0]) < fabs(candidateEndHeadingAngleToTurn_[1]))
            driveDeque.back()->params.float1 = candidateEndHeadingAngleToTurn_[0];
        else
            driveDeque.back()->params.float1 = candidateEndHeadingAngleToTurn_[1];
    }
    if(pushedToFront_) initDequesFront();
}

int DriveRelative::run()
{
	return runDeques();
}
