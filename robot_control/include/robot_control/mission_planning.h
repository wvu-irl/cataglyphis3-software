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

#ifndef MISSION_PLANNING_H
#define MISSION_PLANNING_H
#include <ros/ros.h>
#include "mission_planning_procedure_share.h"
#include <messages/RobotPose.h>
#include <messages/ExecActionEnded.h>
#include <messages/nb1_to_i7_msg.h>
#include <messages/nb2_3_to_i7_msg.h>
#include <messages/EmergencyEscapeTrigger.h>
#include <messages/MissionPlanningInfo.h>
#include <messages/MissionPlanningControl.h>
#include <messages/NavFilterOut.h>
#include "initialize.h"
#include "emergency_escape.h"
#include "avoid.h"
#include "bias_removal.h"
#include "next_best_region.h"
#include "search_region.h"
#include "examine.h"
#include "approach.h"
#include "collect.h"
#include "confirm_collect.h"
#include "reorient.h"
#include "go_home.h"
#include "square_update.h"
#include "deposit_approach.h"
#include "deposit_sample.h"
#include "safe_mode.h"
#include "sos_mode.h"
#include "pause.h"
#include "bit_utils.h"

class MissionPlanning : public MissionPlanningProcedureShare
{
public:
	// Methods
	MissionPlanning();
	void run();
	// Members
	ros::NodeHandle nh;
	ros::Publisher infoPub;
	ros::Subscriber poseSub;
	ros::Subscriber ExecActionEndedSub;
    ros::Subscriber nb1Sub;
    ros::Subscriber nb2Sub;
	ros::Subscriber collisionSub;
	ros::Subscriber navSub;
	ros::ServiceServer emergencyEscapeServ;
	ros::ServiceServer controlServ;
	messages::MissionPlanningInfo infoMsg;
    messages::nb1_to_i7_msg nb1Msg;
	const int loopRate = 20; // Hz
	Initialize initialize;
	EmergencyEscape emergencyEscape;
	Avoid avoid;
	BiasRemoval biasRemoval;
	NextBestRegion nextBestRegion;
	SearchRegion searchRegion;
	Examine examine;
	Approach approach;
	Collect collect;
	ConfirmCollect confirmCollect;
	Reorient reorient;
	GoHome goHome;
	SquareUpdate squareUpdate;
	DepositApproach depositApproach;
	DepositSample depositSample;
	SafeMode safeMode;
	SosMode sosMode;
    Pause pause;

	bool collisionInterruptTrigger;
	Leading_Edge_Latch collisionInterruptLEL;
	bool multiProcLockout;
	unsigned int lockoutSum;
	bool initComplete;
	bool pauseStarted;
	const float homeX = 5.0; // m
	const float homeY = 0.0; // m
	float avoidRemainingWaypointDistance;
	bool shouldExecuteAvoidManeuver;
private:
	void evalConditions_();
	void runProcedures_();
	void runPause_();
	void pauseAllTimers_();
	void resumeTimers_();
	void serviceSearchTimer_();
	void calcnumProcsBeingOrToBeExecOrRes_();
	//void updateSampleFlags_();
	void packAndPubInfoMsg_();
	void poseCallback_(const messages::RobotPose::ConstPtr& msg);
	void ExecActionEndedCallback_(const messages::ExecActionEnded::ConstPtr& msg);
    void nb1Callback_(const messages::nb1_to_i7_msg::ConstPtr& msg);
    void nb2Callback_(const messages::nb2_3_to_i7_msg::ConstPtr& msg);
	void collisionCallback_(const messages::CollisionOut::ConstPtr& msg);
	void navCallback_(const messages::NavFilterOut::ConstPtr& msg);
	void execInfoCallback_(const messages::ExecInfo::ConstPtr& msg);
	void cvSamplesCallback_(const messages::CVSamplesFound::ConstPtr& msg);
	void lidarFilterCallback_(const messages::LidarFilterOut::ConstPtr& msg);
	void hsmMasterStatusCallback_(const messages::MasterStatus::ConstPtr& msg);
	void nextWaypointCallback_(const messages::NextWaypointOut::ConstPtr& msg);
	void grabberStatusCallback_(const messages::ExecGrabberStatus::ConstPtr& msg);
	bool emergencyEscapeCallback_(messages::EmergencyEscapeTrigger::Request &req, messages::EmergencyEscapeTrigger::Response &res);
	bool controlCallback_(messages::MissionPlanningControl::Request &req, messages::MissionPlanningControl::Response &res);
	void roiTimeExpiredCallback_(const ros::TimerEvent &event);
	void biasRemovalTimerCallback_(const ros::TimerEvent& event);
	void homingTimerCallback_(const ros::TimerEvent& event);
	void searchTimerCallback_(const ros::TimerEvent& event);
	void biasRemovalActionTimerCallback_(const ros::TimerEvent &event);
	void queueEmptyTimerCallback_(const ros::TimerEvent &event);
	void roiOvertimeTimerCallback_(const ros::TimerEvent &event);
};

#endif // MISSION_PLANNING_H
