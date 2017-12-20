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

#ifndef EXEC_H
#define EXEC_H
#include <ros/ros.h>
#include <deque>
#include "action_type_enum.h"
#include "action.h"
#include "action_params.h"
#include "idle.h"
#include "halt.h"
#include "drive_global.h"
#include "drive_relative.h"
#include "drop.h"
#include "grab.h"
#include "open.h"
#include "search.h"
#include "wait.h"
#include <messages/ExecAction.h>
#include <messages/ActuatorOut.h>
#include <messages/ExecInfo.h>
#include <messages/ExecActionEnded.h>
#include <messages/RobotPose.h>
#include <messages/NavFilterOut.h>
#include <messages/GrabberFeedback.h>
#include <messages/NextWaypointOut.h>
#include <messages/encoder_data.h>
#include <messages/ExecManualOverride.h>
#include <messages/ExecGrabberStatus.h>
#include <robot_control/DriveSpeeds.h>

#define ACTION_POOL_SIZE 100

class Exec : public RobotControlInterface
{
public:
	// Members
	ros::NodeHandle nh;
	ros::Publisher infoPub;
	ros::Publisher actuatorPub;
	ros::Publisher actionEndedPub;
	ros::Publisher nextWaypointOutPub;
	ros::Publisher grabberStatusPub;
	ros::ServiceServer actionServ;
	ros::ServiceServer manualOverrideServ;
	ros::Subscriber poseSub;
	ros::Subscriber navSub;
	ros::Subscriber grabberSub;
	ros::Subscriber driveSpeedsSub;
	ros::Subscriber leftRoboteqSub;
	ros::Subscriber rightRoboteqSub;

	const int loopRate = 20; // Hz
	// Methods
	Exec(); // Constructor
	void run(); // Main run method for exec
private:
	// Members
	std::deque <Action*> actionDeque_;
	Idle pauseIdle_;
	ACTION_TYPE_T nextActionType_ = _idle;
	bool newActionFlag_ = false;
	bool pushToFrontFlag_ = false;
	bool clearDequeFlag_ = false;
	bool clearFrontFlag_ = false;
	bool pause_ = true;
	bool pausePrev_ = true;
	bool manualOverride_ = false;
	bool actionDequeEmptyPrev_;
	int currentActionDone_ = 0;
	size_t actionDequeSize_ = 0;
	unsigned int actionPoolIndex_[NUM_ACTIONS];
	Action* actionPool_[NUM_ACTIONS][ACTION_POOL_SIZE];
	ACTION_PARAMS_T params_;
	messages::ActuatorOut actuatorMsgOut_;
	messages::ExecInfo execInfoMsgOut_;
	messages::ExecActionEnded execActionEndedMsgOut_;
	messages::NextWaypointOut nextWaypointMsgOut_;
	messages::ExecGrabberStatus grabberStatusMsgOut_;
	double execStartTime_;
	double execElapsedTime_;
	// Methods
	bool actionCallback_(messages::ExecAction::Request &req, messages::ExecAction::Response &res);
	bool manualOverrideCallback_(messages::ExecManualOverride::Request &req, messages::ExecManualOverride::Response &res);
	void poseCallback_(const messages::RobotPose::ConstPtr& msg);
	void navCallback_(const messages::NavFilterOut::ConstPtr& msg);
	void grabberCallback_(const messages::GrabberFeedback::ConstPtr& msg);
	void driveSpeedsCallback_(const robot_control::DriveSpeeds::ConstPtr& msg);
	void leftRoboteqCallback_(const messages::encoder_data::ConstPtr& msg);
	void rightRoboteqCallback_(const messages::encoder_data::ConstPtr& msg);
	void packActuatorMsgOut_();
	void packInfoMsgOut_();
	void packNextWaypointOut_();
	void packGabberStatusOut_();
};

#endif // EXEC_H
