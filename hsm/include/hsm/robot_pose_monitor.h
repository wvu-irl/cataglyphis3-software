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

#ifndef ROBOT_POSE_MONITOR_H
#define ROBOT_POSE_MONITOR_H
#include <ros/ros.h>
#include <messages/RobotPose.h>
#include <messages/NavFilterOut.h>
#include <messages/SLAMPoseOut.h>

class RobotPoseMonitor
{
public:
	// members
	ros::NodeHandle nh;
	ros::Publisher bestPosePub;
	ros::Subscriber navSub;
	ros::Subscriber slamSub;
	messages::RobotPose bestPoseMsg;
	messages::NavFilterOut navMsg;
	messages::SLAMPoseOut slamMsg;
	ros::Timer poseMonitorTimer;
	const float poseMonitorPeriod = 0.05;
	float navFilterConf;
	float slamConf;
	float navSolutionError;
	bool divergenceDetected;
	bool navSolutionsDiverged;
	double divergenceStartTime;
	const float divergenceTriggerTime = 5.0; // sec
	const float divergenceTriggerDistance = 15.0; // m
	enum DIVERGENCE_STATE_T {__notDiverged__, __diverged__} divergenceState;
	// methods
	RobotPoseMonitor();
	void serviceMonitor(const ros::TimerEvent&);
	void navCallback(const messages::NavFilterOut::ConstPtr& msg);
	void slamCallback(const messages::SLAMPoseOut::ConstPtr& msg);
};

#endif // ROBOT_POSE_MONITOR_H
