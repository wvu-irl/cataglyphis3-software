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

#ifndef GRABBER_SET_SLIDES_H
#define GRABBER_SET_SLIDES_H
#include "task.h"

class GrabberSetSlides : public Task
{
public:
	void init();
	int run();
private:
	int slidePos_;
	int slideRecoveryPos_;
	int numFailedAttempts_;
	int slidesFinishedCount_;
	int slidesStuckOpenWaitCount_;
	const int maxSlidesFinishedCount_ = 300; // 15 sec = 300 counts at 20 hz
	const int maxSlidesStuckOpenWaitCount_ = 200; //  10 sec = 200 counts at 2- hz
	const int maxFailedAttempts_ = 2;
	const int limitedRetryPosition_ = GRABBER_CLOSED;
	const int failsafePosition_ = GRABBER_OPEN;
	bool limitedRetry_;
	enum SLIDES_STATE_T {_normal_, _recovering_, _stuckOpenWait_} state_;
	int returnValue_;
};

#endif // GRABBER_SET_SLIDES_H
