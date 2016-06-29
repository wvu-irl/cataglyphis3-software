#ifndef MISSION_PLANNING_H
#define MISSION_PLANNING_H
#include <ros/ros.h>
#include "mission_planning_process_share.h"
#include <messages/NavFilterOut.h>
#include <messages/ExecActionEnded.h>
#include <messages/nb1_to_i7_msg.h>
#include "avoid.h"
#include "next_best_region.h"
#//include "search_region.h"
//#include "examine.h"
#include "approach.h"
#include "collect.h"
#include "confirm_collect.h"
#include "go_home.h"
#include "deposit_approach.h"
#include "deposit_sample.h"
#include "pause.h"
#include "bit_utils.h"

class MissionPlanning : public MissionPlanningProcessShare
{
public:
	// Methods
	MissionPlanning();
	void run();
	// Members
	ros::NodeHandle nh;
	ros::Subscriber navSub;
	ros::Subscriber ExecActionEndedSub;
    ros::Subscriber nb1Sub;
	ros::Subscriber collisionSub;
    messages::nb1_to_i7_msg nb1Msg;
	const int loopRate = 20; // Hz
	Avoid avoid;
	NextBestRegion nextBestRegion;
	//SearchRegion searchRegion;
	Approach approach;
	Collect collect;
	ConfirmCollect confirmCollect;
	GoHome goHome;
	DepositApproach depositApproach;
	DepositSample depositSample;
    Pause pause;



	bool collisionInterruptTrigger;
	Leading_Edge_Latch collisionInterruptLEL;
	unsigned int numProcsBeingExec;
	bool multiProcLockout;
	unsigned int lockoutSum;
	bool initComplete;
	bool pauseStarted;
	const float homeX = 5.0; // m
	const float homeY = 0.0; // m
	const float collisionDistanceThresh = 5.0; // m
private:
	void evalConditions_();
	void runProcesses_();
	void runPause_();
	void calcNumProcsBeingExec_();
	void updateSampleFlags_();
	void navCallback_(const messages::NavFilterOut::ConstPtr& msg);
	void ExecActionEndedCallback_(const messages::ExecActionEnded::ConstPtr& msg);
    void nb1Callback_(const messages::nb1_to_i7_msg::ConstPtr& msg);
	void collisionCallback_(const messages::CollisionOut::ConstPtr& msg);
	void execInfoCallback_(const messages::ExecInfo::ConstPtr& msg);
	void cvSamplesCallback_(const messages::CVSamplesFound::ConstPtr& msg);
};

#endif // MISSION_PLANNING_H
