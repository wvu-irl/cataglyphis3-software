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

#ifndef DRIVE_STRAIGHT_H
#define DRIVE_STRAIGHT_H
#include "task.h"

class DriveStraight : public Task
{
public:
	void init();
	int run();
private:
	float initX_;
	float initY_;
	float initHeading_;
	int driveSign_;
	int pivotSign_;
	float desiredDistance_;
	float remainingDistance_;
	float traversedDistance_;
	float deltaHeading_;
	float vMax_;
	float vDesRaw_;
	float vDesCoerc_;
	float rDes_;
	float errorR_;
	float yawRatePrev_;
	int leftSpeed_;
	int rightSpeed_;
	float headingErrorSpeedT_;
	float headingErrorSpeedP_;
	float headingErrorSpeedI_;
	unsigned int timeoutValue_;
	unsigned int timeoutCounter_;
	int taskEnded_;
	const float vMin_ = 0.03; // m/s
	const float kpV_ = 1.2; // m/(s*m)
	const float kVOutput_ = 900/1.2; // 90% of max speed at 1.2 m/s
	const float kpR_ = 1.5; // deg/(s*deg)
	const float kiR_ = 0.1;
	const float kROutput_ = 450/45.0; // 45% of max speed at 45 deg/s
	const float rMax_ = 30.0; // deg/s
	const float maxHeadingErrorSpeed_ = 50.0;
	const float maxHeadingErrorSpeedI_ = 30.0;
};

#endif // DRIVE_STRAIGHT_H
