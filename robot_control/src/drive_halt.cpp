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

#include <robot_control/drive_halt.h>

void DriveHalt::init()
{
	robotOutputs.flMotorSpeed = 0;
	robotOutputs.mlMotorSpeed = 0;
	robotOutputs.blMotorSpeed = 0;
	robotOutputs.frMotorSpeed = 0;
	robotOutputs.mrMotorSpeed = 0;
	robotOutputs.brMotorSpeed = 0;
    robotOutputs.stopFlag = false;
    robotOutputs.turnFlag = false;
    stopCounts_ = 0;
    speedI_ = 0.0;
    posError_ = 0.0;
    vPrev_ = robotStatus.velocity;
    prevTime_ = ros::Time::now().toSec();
    if(fabs(robotStatus.pitchAngle)>minTiltForHold_) {state_ = _waitingForStop; ROS_WARN("tilt at halt = %f",robotStatus.pitchAngle);}
    else state_ = _noHold;
}

int DriveHalt::run()
{
    switch(state_)
    {
    case _noHold:
    	ROS_INFO("_noHold");
        robotOutputs.flMotorSpeed = 0;
        robotOutputs.mlMotorSpeed = 0;
        robotOutputs.blMotorSpeed = 0;
        robotOutputs.frMotorSpeed = 0;
        robotOutputs.mrMotorSpeed = 0;
        robotOutputs.brMotorSpeed = 0;
        robotOutputs.stopFlag = true;
        robotOutputs.turnFlag = false;
        state_ = _noHold;
        break;
    case _waitingForStop:
    	ROS_INFO("_waitingForStop");
        robotOutputs.flMotorSpeed = 0;
        robotOutputs.mlMotorSpeed = 0;
        robotOutputs.blMotorSpeed = 0;
        robotOutputs.frMotorSpeed = 0;
        robotOutputs.mrMotorSpeed = 0;
        robotOutputs.brMotorSpeed = 0;
        robotOutputs.stopFlag = false;
        robotOutputs.turnFlag = false;
        if(fabs(robotStatus.velocity)<=stopVelocityThreshold_) stopCounts_++;
        else stopCounts_ = 0;
        if(stopCounts_>=stopCountsThreshold_) state_ = _holding;
        else state_ = _waitingForStop;
        prevTime_ = ros::Time::now().toSec();
        vPrev_ = robotStatus.velocity;
        break;
    case _holding:
    	ROS_INFO("_holding");
        if(fabs(posError_>=maxErrorThreshold_))
        {
            posError_ = 0.0;
            speedI_ = 0.0;
        }
        vCurrent_ = robotStatus.velocity;
        if(vCurrent_>vLimit_) vCurrent_ = vLimit_;
        else if(vCurrent_<-vLimit_) vCurrent_ = -vLimit_;
        posError_ += (vCurrent_+vPrev_)/2.0*(ros::Time::now().toSec()-prevTime_);
        vDes_ = -kV_*posError_;
        speedP_ = kpSpeed_*vDes_;
        speedI_ -= kiSpeed_*posError_;
        if(speedI_>speedIMax_) speedI_ = speedIMax_;
        else if(speedI_<-speedIMax_) speedI_ = -speedIMax_;
        speedT_ = speedP_ + speedI_;
        if(speedT_>speedTMax_) speedT_ = speedTMax_;
        else if(speedT_<-speedTMax_) speedT_ = -speedTMax_;
        speedOut_ = round(speedT_);
        robotOutputs.flMotorSpeed = speedOut_;
        robotOutputs.frMotorSpeed = speedOut_;
        robotOutputs.mlMotorSpeed = speedOut_;
        robotOutputs.mrMotorSpeed = speedOut_;
        robotOutputs.blMotorSpeed = speedOut_;
        robotOutputs.brMotorSpeed = speedOut_;
        robotOutputs.stopFlag = false;
        robotOutputs.turnFlag = false;
        vPrev_ = robotStatus.velocity;
        prevTime_ = ros::Time::now().toSec();
        state_ = _holding;
        break;
    }
    ROS_INFO("posError_ = %f",posError_);
	return 1;
}
