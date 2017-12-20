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

#ifndef DRIVE_HALT_H
#define DRIVE_HALT_H
#include "task.h"

class DriveHalt : public Task
{
public:
	void init();
	int run();
private:
	enum DRIVE_HALT_STATES_T {_noHold, _waitingForStop, _holding} state_ = _noHold;
	const float minTiltForHold_ = 6.0; // deg
	double dt_;
	double prevTime_;
	float posError_;
	float vCurrent_;
	float vPrev_;
	float vDes_;
	float speedP_;
	float speedI_;
	float speedT_;
	int stopCounts_;
	int speedOut_;
	const float kV_ = 0.5;
	const float kpSpeed_ = 900.0;
	const float kiSpeed_ = 0.15;
	const float vLimit_ = 1.2; // m/s
	const float maxErrorThreshold_ = 0.2; // m
	const float speedIMax_ = 200.0;
	const float speedTMax_ = 300.0;
	const float stopVelocityThreshold_ = 0.01; // m/s
	const int stopCountsThreshold_ = 40; // counts @ 20 Hz
};

#endif // DRIVE_HALT_H
