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

#ifndef DRIVE_PIVOT_H
#define DRIVE_PIVOT_H
#include "task.h"

class DrivePivot : public Task
{
public:
	void init();
	int run();
private:
	float desiredDeltaHeading_;
	float deltaHeading_;
	float rMax_;
	float initHeading_;
	float rDes_;
	float errorR_;
	float yawRatePrev_;
	int pivotSign_;
	float leftSpeed_;
	float rightSpeed_;
	float rSpeedT_;
	float rSpeedP_;
	float rSpeedI_;
	unsigned int timeoutValue_;
	unsigned int timeoutCounter_;
	double thresholdInitTime_;
	double thresholdTime_;
	bool inThreshold_;
	int taskEnded_;
	const float kpR_ = 0.45; // deg/(s*deg)
	const float kiR_ = 0.1;
	const float rSpeedIMax_ = 50.0;
	const float kROutput_ = 450/45.0; // 45% of max speed at 45 deg/s
	const int rSpeedMax_ = 500;
	const float deltaHeadingThreshold_ = 2.0; // deg
	const double thresholdMinTime_ = 0.25; // s
	const float middleWheelReduction_ = 0.65;
	const float cornerBoostGain_ = 1.2;
	const float reverseMiddleGain_ = 0.8;
	float ccwBoostGain_;
	float cwBoostGain_;
	float leftMiddleWheelMultiplier_;
	float rightMiddleWheelMultiplier_;
	void dogLeg_();
	double dogLegDetectTime_;
	double dogLegStopTime_;
	double dogLegRecoverStartTime_;
	const double dogLegDetectThreshold_ = 2.0; // sec
	const double dogLegStopDuration_ = 0.5; // sec
	const double dogLegRecoverDuration_ = 0.75; // sec
	const long int encoderDogLegTriggerValue_ = 110;
	long int encPrev_[6];
	long int encDelta_[6];
	std::vector<long int> leftDeltaVector_;
	std::vector<long int> rightDeltaVector_;
	const size_t rollingAverageSize_ = 20;
	long int leftDeltaAverage_;
	long int rightDeltaAverage_;
	long int maxLeftDelta_;
	long int maxRightDelta_;
	long int minLeftDelta_;
	long int minRightDelta_;
    long int leftMaxMinusMin_;
    long int rightMaxMinusMin_;
    const long int maxMinusMinLimit_ = 300;
	const float dogLegRDes_ = 45.0;
	bool dogLegDetectTimeStarted_;
	enum DOG_LEG_STATE_T {_monitoring, _stoppingFirst, _recovering, _stoppingSecond} dogLegState;
};

#endif // DRIVE_PIVOT_H
