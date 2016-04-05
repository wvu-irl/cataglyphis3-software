#include <robot_control/exec.h>

Exec::Exec()
{
	robotStatus.loopRate = loopRate;
    actionServ = nh.advertiseService("control/exec/actionin", &Exec::actionCallback_, this);
	navSub = nh.subscribe<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout", 1, &Exec::navCallback_, this);
	grabberSub = nh.subscribe<messages::GrabberFeedback>("roboteq/grabberin/grabberin", 1, &Exec::grabberCallback_, this);
	actuatorPub = nh.advertise<messages::ActuatorOut>("control/actuatorout/all",1);
	infoPub = nh.advertise<messages::ExecInfo>("control/exec/info",1);
    actionEndedPub = nh.advertise<messages::ExecActionEnded>("control/exec/actionended",1);
    cvSearchCmdClient = nh.serviceClient<messages::CVSearchCmd>("/vision/samplesearch/searchforsamples");
	// "allocate" deque memory
    for(int i=0; i<NUM_ACTIONS; i++)
    {
        actionPoolIndex_[i] = 0;
    }
	for(int j=0; j<ACTION_POOL_SIZE; j++)
    {
        actionPool_[_idle][j] = new Idle;
		actionPool_[_halt][j] = new Halt;
		actionPool_[_driveGlobal][j] = new DriveGlobal;
		actionPool_[_driveRelative][j] = new DriveRelative;
		actionPool_[_grab][j] = new Grab;
		actionPool_[_drop][j] = new Drop;
		actionPool_[_open][j] = new Open;
        actionPool_[_search][j] = new Search;
	}
	for(int k=0; k<NUM_TASKS; k++)
	{
		pauseIdle_.taskPoolIndex[k] = 0;
	}
	for(int l=0; l<TASK_POOL_SIZE; l++)
	{
		pauseIdle_.taskPool[_driveHalt_][l] = new DriveHalt;
		pauseIdle_.taskPool[_driveStraight_][l] = new DriveStraight;
		pauseIdle_.taskPool[_pivot_][l] = new DrivePivot;
		pauseIdle_.taskPool[_driveStraightCL_][l] = new DriveStraightCL;
		pauseIdle_.taskPool[_grabberHalt_][l] = new GrabberHalt;
		pauseIdle_.taskPool[_grabberIdle_][l] = new GrabberIdle;
		pauseIdle_.taskPool[_grabberSetDrop_][l] = new GrabberSetDrop;
		pauseIdle_.taskPool[_grabberSetSlides_][l] = new GrabberSetSlides;
		pauseIdle_.taskPool[_visionHalt_][l] = new VisionHalt;
        pauseIdle_.taskPool[_search_][l] = new VisionSearch;
		// remaining vision tasks
	}
    execStartTime_ = ros::Time::now().toSec();
}

void Exec::run()
{
    if(clearDequeFlag_) actionDeque_.clear(); // Clear deque
    if(clearFrontFlag_) currentActionDone_ = 1;
    if(newActionFlag_) // New action to be added to deque
    {
        if(pushToFrontFlag_) // Push new action to front of deque
        {
            actionDeque_.push_front(actionPool_[nextActionType_][actionPoolIndex_[nextActionType_]]); // Push new action pointer of specified type to front of deque
            actionDeque_.front()->params = params_; // Assign params into action object just pushed
        }
        else // Push new action to back of deque
        {
            actionDeque_.push_back(actionPool_[nextActionType_][actionPoolIndex_[nextActionType_]]); // Push new action pointer of specified type to back of deque
            actionDeque_.back()->params = params_; // Assign params into action object just pushed
        }
        actionPoolIndex_[nextActionType_]++; // Increment the action pool index of the action type just pushed
        if(actionPoolIndex_[nextActionType_]>=ACTION_POOL_SIZE) actionPoolIndex_[nextActionType_] = 0; // If pool index has wrapped around, restart at 0
    }
	if(pause_==true && pausePrev_==false) pauseIdle_.visionHalt.init(); // Call vision empty halt init to record camera location for halt
	if(pause_) pauseIdle_.run(); // If pause switch is true, run pause action
    else // Else, run actions from deque
    {
        if(actionDeque_.empty()) // Check if deque is empty
        {
			pauseIdle_.run(); // Perform pause action to halt the robot
            currentActionDone_ = 0;
			//ROS_DEBUG("after deque empty pauseIdle.run");
			//if(newActionFlag_ && !pushToFrontFlag_) actionDeque_.front()->init();  // If a new action has been pushed to the back of the deque, run init() on the new action
        }
        else // Else, deque is not empty and the action at the front() should be run
        {
			if(pushToFrontFlag_ || (newActionFlag_ && actionDeque_.size()==1)) actionDeque_.front()->init(); // If new action was pushed to the front or deque was empty and became not empty, need to run init() for new action
            if(currentActionDone_) // If the action at the front of the deque is complete, pop this action off and init() the next action in the deque
            {
                execActionEndedMsgOut_.procType = static_cast<uint8_t>(actionDeque_.front()->params.procType);
                execActionEndedMsgOut_.serialNum = actionDeque_.front()->params.serialNum;
                actionDeque_.pop_front();
                if(actionDeque_.empty()==0) {actionDeque_.front()->init(); execActionEndedMsgOut_.dequeEmpty = false;}
                else execActionEndedMsgOut_.dequeEmpty = true;
                actionEndedPub.publish(execActionEndedMsgOut_);
            }
			if(actionDeque_.empty()==0) currentActionDone_ = actionDeque_.front()->run();
			else currentActionDone_ = 0;
        }
    }
    clearDequeFlag_ = false;
    newActionFlag_ = false;
    pushToFrontFlag_ = false;
	pausePrev_ = pause_;
	packActuatorMsgOut_();
	packInfoMsgOut_();
	actuatorPub.publish(actuatorMsgOut_);
	infoPub.publish(execInfoMsgOut_);
    execElapsedTime_ = ros::Time::now().toSec() - execStartTime_;
    ROS_INFO_THROTTLE(3,"execElapsedTime = %f",execElapsedTime_);
}

bool Exec::actionCallback_(messages::ExecAction::Request &req, messages::ExecAction::Response &res)
{
    nextActionType_ = static_cast<ACTION_TYPE_T>(req.nextActionType);
    newActionFlag_ = req.newActionFlag;
    pushToFrontFlag_ = req.pushToFrontFlag;
    clearDequeFlag_ = req.clearDequeFlag;
    pause_ = req.pause;
	params_.actionType = nextActionType_;
    params_.float1 = req.float1;
    params_.float2 = req.float2;
    params_.float3 = req.float3;
    params_.float4 = req.float4;
    params_.float5 = req.float5;
    params_.int1 = req.int1;
    params_.bool1 = req.bool1;
    params_.bool2 = req.bool2;
    params_.bool3 = req.bool3;
    params_.bool4 = req.bool4;
    params_.bool5 = req.bool5;
    params_.bool6 = req.bool6;
    params_.bool7 = req.bool7;
    params_.procType = static_cast<PROC_TYPES_T>(req.procType);
    params_.serialNum = req.serialNum;
	ROS_INFO("ACTION CALLBACK, float1 = %f, float2 = %f",params_.float1,params_.float2);
    return true;
}

void Exec::navCallback_(const messages::NavFilterOut::ConstPtr& msg)
{
	robotStatus.xPos = msg->x_position;
	robotStatus.yPos = msg->y_position;
	robotStatus.heading = msg->heading;
	robotStatus.bearing = msg->bearing;
	robotStatus.yawRate = msg->yaw_rate;
}

void Exec::grabberCallback_(const messages::GrabberFeedback::ConstPtr& msg)
{
	robotStatus.grabberSlideStatus = msg->slideStatus;
	robotStatus.grabberDropStatus = msg->dropStatus;
	robotStatus.grabberSlidePos = msg->sliderPosAvg;
	robotStatus.grabberDropPos = msg->dropPos;
}

void Exec::packActuatorMsgOut_()
{
	actuatorMsgOut_.fl_speed_cmd = robotOutputs.flMotorSpeed;
	actuatorMsgOut_.fr_speed_cmd = robotOutputs.frMotorSpeed;
	actuatorMsgOut_.ml_speed_cmd = robotOutputs.mlMotorSpeed;
	actuatorMsgOut_.mr_speed_cmd = robotOutputs.mrMotorSpeed;
	actuatorMsgOut_.bl_speed_cmd = robotOutputs.blMotorSpeed;
	actuatorMsgOut_.br_speed_cmd = robotOutputs.brMotorSpeed;
	actuatorMsgOut_.slide_pos_cmd = robotOutputs.slidePosCmd;
	actuatorMsgOut_.drop_pos_cmd = robotOutputs.dropPosCmd;
	actuatorMsgOut_.grabber_stop_cmd = robotOutputs.grabberStopCmd;
}

void Exec::packInfoMsgOut_()
{
	execInfoMsgOut_.actionDequeSize = actionDeque_.size();
	execInfoMsgOut_.pause = pause_;
    execInfoMsgOut_.stopFlag = robotOutputs.stopFlag;
    execInfoMsgOut_.turnFlag = robotOutputs.turnFlag;
	for(unsigned int i=0; i<100; i++)
	{
		execInfoMsgOut_.actionDeque[i] = 0;
		execInfoMsgOut_.actionFloat1[i] = 0;
		execInfoMsgOut_.actionFloat2[i] = 0;
		execInfoMsgOut_.actionFloat3[i] = 0;
		execInfoMsgOut_.actionFloat4[i] = 0;
		execInfoMsgOut_.actionFloat5[i] = 0;
		execInfoMsgOut_.actionInt1[i] = 0;
		execInfoMsgOut_.actionBool1[i] = 0;
        execInfoMsgOut_.actionBool2[i] = 0;
        execInfoMsgOut_.actionBool3[i] = 0;
        execInfoMsgOut_.actionBool4[i] = 0;
        execInfoMsgOut_.actionBool5[i] = 0;
        execInfoMsgOut_.actionBool6[i] = 0;
        execInfoMsgOut_.actionBool7[i] = 0;
        execInfoMsgOut_.actionProcType[i] = 0;
        execInfoMsgOut_.actionSerialNum[i] = 0;
	}
	for(unsigned int i=0; i<execInfoMsgOut_.actionDequeSize; i++)
	{
		execInfoMsgOut_.actionDeque[i] = actionDeque_.at(i)->params.actionType;
		execInfoMsgOut_.actionFloat1[i] = actionDeque_.at(i)->params.float1;
		execInfoMsgOut_.actionFloat2[i] = actionDeque_.at(i)->params.float2;
		execInfoMsgOut_.actionFloat3[i] = actionDeque_.at(i)->params.float3;
		execInfoMsgOut_.actionFloat4[i] = actionDeque_.at(i)->params.float4;
		execInfoMsgOut_.actionFloat5[i] = actionDeque_.at(i)->params.float5;
		execInfoMsgOut_.actionInt1[i] = actionDeque_.at(i)->params.int1;
		execInfoMsgOut_.actionBool1[i] = actionDeque_.at(i)->params.bool1;
        execInfoMsgOut_.actionBool2[i] = actionDeque_.at(i)->params.bool2;
        execInfoMsgOut_.actionBool3[i] = actionDeque_.at(i)->params.bool3;
        execInfoMsgOut_.actionBool4[i] = actionDeque_.at(i)->params.bool4;
        execInfoMsgOut_.actionBool5[i] = actionDeque_.at(i)->params.bool5;
        execInfoMsgOut_.actionBool6[i] = actionDeque_.at(i)->params.bool6;
        execInfoMsgOut_.actionBool7[i] = actionDeque_.at(i)->params.bool7;
        execInfoMsgOut_.actionProcType[i] = static_cast<uint8_t>(actionDeque_.at(i)->params.procType);
        execInfoMsgOut_.actionSerialNum[i] = actionDeque_.at(i)->params.serialNum;
	}
}
