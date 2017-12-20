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

#include <robot_control/drive_pivot.h>

void DrivePivot::init()
{
	leftDeltaVector_.clear();
	rightDeltaVector_.clear();
	initHeading_ = robotStatus.heading;
	desiredDeltaHeading_ = params.float1;
	if(deltaHeading_<0.0) pivotSign_ = -1;
	else pivotSign_ = 1;
    timeoutValue_ = (unsigned int)round((10.0 + fabs(desiredDeltaHeading_)/10.0)*robotStatus.loopRate);
	timeoutCounter_ = 0;
	rSpeedI_ = 0.0;
	inThreshold_ = false;
	thresholdTime_ = 0.0;
    robotOutputs.stopFlag = false;
    robotOutputs.turnFlag = true;
    encPrev_[0] = robotStatus.flEncoder;
    encPrev_[1] = robotStatus.mlEncoder;
    encPrev_[2] = robotStatus.blEncoder;
    encPrev_[3] = robotStatus.frEncoder;
    encPrev_[4] = robotStatus.mrEncoder;
    encPrev_[5] = robotStatus.brEncoder;
    yawRatePrev_ = 0.0;
    dogLegState = _monitoring;
    dogLegDetectTimeStarted_ = false;
    ROS_DEBUG("drivePivot init");
}

int DrivePivot::run()
{
    //int temp;
    ROS_INFO_THROTTLE(1,"drive pivot running");
    //ROS_INFO("======================");
    robotOutputs.stopFlag = false;
    robotOutputs.turnFlag = true;
    rMax_ = robotStatus.rMax;
	deltaHeading_ = robotStatus.heading - initHeading_;
    //ROS_INFO("desiredDeltaHeading = %f", desiredDeltaHeading_);
    //ROS_INFO("deltaHeading_ = %f",deltaHeading_);
	rDes_ = kpR_*(desiredDeltaHeading_-deltaHeading_);
    //ROS_INFO("rDes before dogLeg = %f",rDes_);
    dogLeg_();
    //ROS_INFO("rDes_ after dogLeg = %f",rDes_);
	if(rDes_>rMax_) rDes_ = rMax_;
	else if(rDes_<(-rMax_)) rDes_ = -rMax_;
    if(rDes_>0.0)
	{
		ccwBoostGain_ = cornerBoostGain_;
		cwBoostGain_ = 1.0;
		rightMiddleWheelMultiplier_ = reverseMiddleGain_;
		leftMiddleWheelMultiplier_ = 1.0;
	}
    else
	{
		ccwBoostGain_ = 1.0;
		cwBoostGain_ = cornerBoostGain_;
		rightMiddleWheelMultiplier_ = 1.0;
		leftMiddleWheelMultiplier_ = reverseMiddleGain_;
	}
    if(std::isnan(robotStatus.yawRate)) {ROS_ERROR("yaw rate is nan"); robotStatus.yawRate = yawRatePrev_;}
    else yawRatePrev_ = robotStatus.yawRate;
    //ROS_INFO("yawRate = %f",robotStatus.yawRate);
	errorR_ = rDes_ - robotStatus.yawRate;
    //ROS_INFO("errorR_ = %f",errorR_);
	rSpeedP_ = kROutput_*rDes_;
    //ROS_INFO("rSpeedP_ = %f",rSpeedP_);
    rSpeedI_ += kiR_*errorR_;
    //ROS_INFO("rSpeedI_ before coerc = %f",rSpeedI_);
    if(rSpeedI_>rSpeedIMax_) rSpeedI_ = rSpeedIMax_;
    else if(rSpeedI_<-rSpeedIMax_) rSpeedI_ = -rSpeedIMax_;
    //ROS_INFO("rSpeedI_ before coerc = %f",rSpeedI_);
    rSpeedT_ = round(rSpeedP_ + rSpeedI_);
    //ROS_INFO("rSpeedT_ before coerc = %f",rSpeedT_);
	if(rSpeedT_>rSpeedMax_) rSpeedT_ = rSpeedMax_;
	else if(rSpeedT_<(-rSpeedMax_)) rSpeedT_ = -rSpeedMax_;
    //ROS_INFO("rSpeedT after coerc = %f",rSpeedT_);
    //ROS_INFO("rDes: %f",rDes_);
    //ROS_INFO("deltaHeading = %f", deltaHeading_);
    //ROS_INFO("desiredDeltaHeading_-deltaHeading_: %f", desiredDeltaHeading_-deltaHeading_);
	leftSpeed_ = rSpeedT_;
	rightSpeed_ = -rSpeedT_;
    //ROS_INFO("leftSpeed_ = %f",leftSpeed_);
    //ROS_INFO("rightSpeed_ = %f",rightSpeed_);
	timeoutCounter_++;
	if(fabs(desiredDeltaHeading_-deltaHeading_)<=deltaHeadingThreshold_ && inThreshold_==false) {thresholdInitTime_ = ros::Time::now().toSec(); inThreshold_ = true;}
	if(fabs(desiredDeltaHeading_-deltaHeading_)<=deltaHeadingThreshold_) thresholdTime_ = ros::Time::now().toSec() - thresholdInitTime_;
	else {thresholdTime_ = 0.0; inThreshold_ = false;}
	if(thresholdTime_ >= thresholdMinTime_ || timeoutCounter_ >= timeoutValue_)
	{
		robotOutputs.flMotorSpeed = 0;
		robotOutputs.mlMotorSpeed = 0;
		robotOutputs.blMotorSpeed = 0;
		robotOutputs.frMotorSpeed = 0;
		robotOutputs.mrMotorSpeed = 0;
		robotOutputs.brMotorSpeed = 0;
        ROS_INFO("end pivot");
		taskEnded_ = 1;
	}
	else
	{
        robotOutputs.flMotorSpeed = (int)round(leftSpeed_*cwBoostGain_);
        robotOutputs.mlMotorSpeed = (int)round(leftSpeed_*middleWheelReduction_*leftMiddleWheelMultiplier_);
        robotOutputs.blMotorSpeed = (int)round(leftSpeed_*ccwBoostGain_);
        robotOutputs.frMotorSpeed = (int)round(rightSpeed_*ccwBoostGain_);
        robotOutputs.mrMotorSpeed = (int)round(rightSpeed_*middleWheelReduction_*rightMiddleWheelMultiplier_);
        robotOutputs.brMotorSpeed = (int)round(rightSpeed_*cwBoostGain_);
		taskEnded_ = 0;
	}
    //ROS_INFO("task ended = %i",taskEnded_);
    //std::printf("\n");
	return taskEnded_;
}

void DrivePivot::dogLeg_()
{
    //ROS_INFO("dogLegState = %i",dogLegState);
    switch(dogLegState)
    {
    case _monitoring:
        encDelta_[0] = abs(robotStatus.flEncoder - encPrev_[0]);
        encDelta_[1] = abs(robotStatus.mlEncoder - encPrev_[1]);
        encDelta_[2] = abs(robotStatus.blEncoder - encPrev_[2]);
        encDelta_[3] = abs(robotStatus.frEncoder - encPrev_[3]);
        encDelta_[4] = abs(robotStatus.mrEncoder - encPrev_[4]);
        encDelta_[5] = abs(robotStatus.brEncoder - encPrev_[5]);
        minLeftDelta_ = 10000;
        maxLeftDelta_ = 0;
        minRightDelta_ = 10000;
        maxRightDelta_ = 0;
        for(int i=0; i<3; i++) // Left side
        {
            if(encDelta_[i] < minLeftDelta_) minLeftDelta_ = encDelta_[i];
            if(encDelta_[i] > maxLeftDelta_) maxLeftDelta_ = encDelta_[i];
        }
        for(int i=3; i<6; i++) // Right side
        {
            if(encDelta_[i] < minRightDelta_) minRightDelta_ = encDelta_[i];
            if(encDelta_[i] > maxRightDelta_) maxRightDelta_ = encDelta_[i];
        }
        leftMaxMinusMin_ = abs(maxLeftDelta_ - minLeftDelta_);
        rightMaxMinusMin_ = abs(maxRightDelta_ - minRightDelta_);
        if(leftMaxMinusMin_ > maxMinusMinLimit_) leftMaxMinusMin_ = maxMinusMinLimit_;
        if(rightMaxMinusMin_ > maxMinusMinLimit_) rightMaxMinusMin_ = maxMinusMinLimit_;
        leftDeltaVector_.insert(leftDeltaVector_.begin(),leftMaxMinusMin_);
        rightDeltaVector_.insert(rightDeltaVector_.begin(),rightMaxMinusMin_);
        if(leftDeltaVector_.size()>rollingAverageSize_) leftDeltaVector_.pop_back();
        if(rightDeltaVector_.size()>rollingAverageSize_) rightDeltaVector_.pop_back();
        leftDeltaAverage_ = 0;
        for(int i=0; i<leftDeltaVector_.size(); i++)
        {
        	leftDeltaAverage_ += leftDeltaVector_.at(i);
        }
        rightDeltaAverage_ = 0;
        for(int i=0; i<rightDeltaVector_.size(); i++)
        {
        	rightDeltaAverage_ += rightDeltaVector_.at(i);
        }
        if(leftDeltaVector_.size()>=rollingAverageSize_) leftDeltaAverage_ /= leftDeltaVector_.size();
        else leftDeltaAverage_ = 1;
        if(rightDeltaVector_.size()>=rollingAverageSize_) rightDeltaAverage_ /= rightDeltaVector_.size();
        else rightDeltaAverage_ = 1;
        ROS_INFO_THROTTLE(1,"left dog leg delta average = %i",leftDeltaAverage_);
        ROS_INFO_THROTTLE(1,"right dog leg delta average = %i",rightDeltaAverage_);
        if((leftDeltaAverage_ > encoderDogLegTriggerValue_ || rightDeltaAverage_ > encoderDogLegTriggerValue_)
                && !dogLegDetectTimeStarted_) {dogLegDetectTimeStarted_ = true; dogLegDetectTime_ = ros::Time::now().toSec();}
        else if((leftDeltaAverage_ <= encoderDogLegTriggerValue_ && rightDeltaAverage_ <= encoderDogLegTriggerValue_)
         		&& dogLegDetectTimeStarted_) dogLegDetectTimeStarted_ = false;
        if(dogLegDetectTimeStarted_ && (ros::Time::now().toSec() - dogLegDetectTime_) >= dogLegDetectThreshold_) {dogLegState = _stoppingFirst; dogLegStopTime_ = ros::Time::now().toSec();}
        else dogLegState = _monitoring;
        break;
    case _stoppingFirst:
        ROS_INFO("dog leg detected, first stopping");
        if((ros::Time::now().toSec() - dogLegStopTime_) > dogLegStopDuration_)
        {
        	rSpeedI_ = 0.0;
        	dogLegRecoverStartTime_ = ros::Time::now().toSec();
        	dogLegState = _recovering;
        }
        else
        {
        	dogLegState = _stoppingFirst;
        }
        rDes_ = 0.0;
        break;
    case _recovering:
    	ROS_INFO("dog leg recovering");
        if((ros::Time::now().toSec() - dogLegRecoverStartTime_) > dogLegRecoverDuration_)
        {
            dogLegStopTime_ = ros::Time::now().toSec();
            dogLegDetectTimeStarted_ = false;
            dogLegState = _stoppingSecond;
        }
        else
        {
        	if(pivotSign_ > 0) rDes_ = -dogLegRDes_;
            else rDes_ = dogLegRDes_;
            dogLegState = _recovering;
        }
        break;
    case _stoppingSecond:
        ROS_INFO("dog leg, second stopping");
    	if((ros::Time::now().toSec() - dogLegStopTime_) > dogLegStopDuration_)
    	{
    		rSpeedI_ = 0.0;
    		dogLegState = _monitoring;
    	}
    	else
    	{
    		dogLegState = _stoppingSecond;
    	}
    	rDes_ = 0.0;
    	break;
    }
    encPrev_[0] = robotStatus.flEncoder;
    encPrev_[1] = robotStatus.mlEncoder;
    encPrev_[2] = robotStatus.blEncoder;
    encPrev_[3] = robotStatus.frEncoder;
    encPrev_[4] = robotStatus.mrEncoder;
    encPrev_[5] = robotStatus.brEncoder;
}
