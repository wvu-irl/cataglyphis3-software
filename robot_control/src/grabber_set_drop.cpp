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

#include <robot_control/grabber_set_drop.h>

void GrabberSetDrop::init()
{
	dropPos_ = params.int1;
    if(dropPos_ == limitedRetryPosition_) limitedRetry_ = true;
    else limitedRetry_ = false;
    state_ = _normal_;
    numFailedAttempts_ = 0;
    dropFailed_ = false;
    dropEnded_ = false;
    dropFinishedCount_ = 0;
}

int GrabberSetDrop::run()
{
    switch(state_)
    {
    case _normal_:
        ROS_INFO("grabber set drop normal state");
        robotOutputs.dropPosCmd = dropPos_;
        robotOutputs.grabberStopCmd = 0;
        if(dropEnded_ && abs(robotStatus.grabberDropPos - dropPos_) <= dropTol_) {returnValue_ = 1; state_ = _normal_;}
        //else if(dropEnded_ && abs(robotStatus.grabberDropPos - dropPos_) > dropTol_) {returnValue_ = 0; dropEnded_ = false; state_ = _recovering_;}
        else {returnValue_ = 0; state_ = _normal_;}
        break;
    case _recovering_:
        ROS_INFO("grabber set drop recovering state");
        if(dropPos_ == GRABBER_DROPPED) dropRecoveryPos_ = GRABBER_RAISED;
        else dropRecoveryPos_ = GRABBER_DROPPED;
        robotOutputs.dropPosCmd = dropRecoveryPos_;
        robotOutputs.grabberStopCmd = 0;
        if(dropEnded_) {returnValue_ = 0; numFailedAttempts_++; state_ = _normal_;}
        else {returnValue_ = 0; state_ = _recovering_;}
        if(limitedRetry_ && (numFailedAttempts_ >= maxFailedAttempts_)) {dropFailed_ = true; returnValue_ = 1;}
        break;
    }
    return returnValue_;
}
