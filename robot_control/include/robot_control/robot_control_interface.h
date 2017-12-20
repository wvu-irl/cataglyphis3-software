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

#ifndef ROBOT_CONTROL_INTERFACE_H
#define ROBOT_CONTROL_INTERFACE_H
#include "robot_outputs.h"
#include "robot_status.h"
#include <messages/CVSearchCmd.h>
#include <ros/ros.h>
#include "bit_utils.h"
#define GRABBER_OPEN 1000
#define GRABBER_CLOSED -900
#define GRABBER_DROPPED 1000
#define GRABBER_RAISED -1000

class RobotControlInterface
{
public:
    static RobotStatus robotStatus;
    static RobotOutputs robotOutputs;
	static ros::ServiceClient cvSearchCmdClient;
	static messages::CVSearchCmd cvSearchCmdSrv;
	static Leading_Edge_Latch dropStatusLEL_;
	static Leading_Edge_Latch slideStatusLEL_;
	static bool dropEnded_;
	static bool slidesEnded_;
	static bool dropFailed_;
	static bool slidesFailed_;
	const int dropTol_ = 300;
	const int slideTol_ = 500;
};

RobotStatus RobotControlInterface::robotStatus;
RobotOutputs RobotControlInterface::robotOutputs;
ros::ServiceClient RobotControlInterface::cvSearchCmdClient;
messages::CVSearchCmd RobotControlInterface::cvSearchCmdSrv;
Leading_Edge_Latch RobotControlInterface::dropStatusLEL_;
Leading_Edge_Latch RobotControlInterface::slideStatusLEL_;
bool RobotControlInterface::dropEnded_;
bool RobotControlInterface::slidesEnded_;
bool RobotControlInterface::dropFailed_;
bool RobotControlInterface::slidesFailed_;

#endif // ROBOT_CONTROL_INTERFACE_H
