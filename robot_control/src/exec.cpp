#include <robot_control/exec.h>

Exec::Exec()
{
	robotStatus.loopRate = loopRate;
    actionServ = nh.advertiseService("control/exec/actionin", &Exec::actionCallback_, this);
    manualOverrideServ = nh.advertiseService("/control/exec/manualoverride", &Exec::manualOverrideCallback_, this);
    poseSub = nh.subscribe<messages::RobotPose>("/hsm/masterexec/globalpose", 1, &Exec::poseCallback_, this);
    navSub = nh.subscribe<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout", 1, &Exec::navCallback_, this);
	grabberSub = nh.subscribe<messages::GrabberFeedback>("roboteq/grabberin/grabberin", 1, &Exec::grabberCallback_, this);
    driveSpeedsSub = nh.subscribe<robot_control::DriveSpeeds>("/control/missionplanning/drivespeeds", 1, &Exec::driveSpeedsCallback_, this);
    leftRoboteqSub = nh.subscribe<messages::encoder_data>("/roboteq/drivemotorin/left", 1, &Exec::leftRoboteqCallback_, this);
    rightRoboteqSub = nh.subscribe<messages::encoder_data>("/roboteq/drivemotorin/right", 1, &Exec::rightRoboteqCallback_, this);
	actuatorPub = nh.advertise<messages::ActuatorOut>("control/actuatorout/all",1);
	infoPub = nh.advertise<messages::ExecInfo>("control/exec/info",1);
    actionEndedPub = nh.advertise<messages::ExecActionEnded>("control/exec/actionended",1);
    nextWaypointOutPub = nh.advertise<messages::NextWaypointOut>("/control/exec/nextwaypoint", 1);
    grabberStatusPub = nh.advertise<messages::ExecGrabberStatus>("/control/exec/grabberstatus", 1);
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
        actionPool_[_wait][j] = new Wait;
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
    actionDequeEmptyPrev_ = true;
}

void Exec::run()
{
    ROS_INFO_THROTTLE(3,"Exec running...");
    dropStatusLEL_.LE_Latch(robotStatus.grabberDropStatus);
    slideStatusLEL_.LE_Latch(robotStatus.grabberSlideStatus);
    if(pause_==false)
    {
        if(dropStatusLEL_.get_val()) dropEnded_ = true;
        if(slideStatusLEL_.get_val()) slidesEnded_ = true;
    }
    else
    {
        if(abs(robotStatus.grabberDropPos - robotOutputs.dropPosCmd) <= dropTol_) dropEnded_ = true;
        if(abs(robotStatus.grabberSlidePos - robotOutputs.slidePosCmd) <= slideTol_) slidesEnded_ = true;
    }
    ROS_INFO("========== ============== ===========");
    ROS_INFO("dropEnded_ = %i",dropEnded_);
    ROS_INFO("slidesEnded_ = %i",slidesEnded_);
    if(clearDequeFlag_) {actionDeque_.clear(); pauseIdle_.clearDeques();} // Clear deques
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
    if(pause_==true && pausePrev_==false) pauseIdle_.driveHalt.init(); // Call init on driveHalt to begin possible drive hold
	if(pause_)
	{
		ROS_INFO("exec pause");
		pauseIdle_.run(); // If pause switch is true, run pause action
		if(pushToFrontFlag_ || (newActionFlag_ && actionDeque_.size()==1)) actionDeque_.front()->init();
	}
    else // Else, run actions from deque
    {
    	ROS_INFO("currentActionDone_ = %i",currentActionDone_);
        //if(actionDeque_.empty() && !actionDequeEmptyPrev_) pauseIdle_.init();
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
                currentActionDone_ = 0;
            }
			if(actionDeque_.empty()==0) currentActionDone_ = actionDeque_.front()->run();
			else currentActionDone_ = 0;
        }
    }
    actionDequeEmptyPrev_ = actionDeque_.empty();
    clearDequeFlag_ = false;
    clearFrontFlag_ = false;
    newActionFlag_ = false;
    pushToFrontFlag_ = false;
	pausePrev_ = pause_;
	packActuatorMsgOut_();
	packInfoMsgOut_();
    packNextWaypointOut_();
    packGabberStatusOut_();
    if(!manualOverride_)
    {
        actuatorPub.publish(actuatorMsgOut_);
        infoPub.publish(execInfoMsgOut_);
    }
    nextWaypointOutPub.publish(nextWaypointMsgOut_);
    grabberStatusPub.publish(grabberStatusMsgOut_);
    std::printf("\n");
    /*execElapsedTime_ = ros::Time::now().toSec() - execStartTime_;
    ROS_INFO("*******\nexecElapsedTime = %f",execElapsedTime_);
    for(int i=0; i<NUM_ACTIONS; i++) ROS_INFO("actionPoolIndex[%i] = %i",i,actionPoolIndex_[i]);
    for(int i=0; i<NUM_TASKS; i++) ROS_INFO("taskPoolIndex[%i] = %i",i,pauseIdle_.taskPoolIndex[i]);
    ROS_INFO("actionDeque size = %u",actionDeque_.size());
    ROS_INFO("driveDeque size = %u",pauseIdle_.driveDeque.size());
    ROS_INFO("grabberDeque size = %u",pauseIdle_.grabberDeque.size());
    ROS_INFO("visionDeque size = %u\n",pauseIdle_.visionDeque.size());*/
}

bool Exec::actionCallback_(messages::ExecAction::Request &req, messages::ExecAction::Response &res)
{
    nextActionType_ = static_cast<ACTION_TYPE_T>(req.nextActionType);
    newActionFlag_ = req.newActionFlag;
    pushToFrontFlag_ = req.pushToFrontFlag;
    clearDequeFlag_ = req.clearDequeFlag;
    clearFrontFlag_ = req.clearFrontFlag;
    if(!req.pauseUnchanged) pause_ = req.pause;
	params_.actionType = nextActionType_;
    params_.float1 = req.float1;
    params_.float2 = req.float2;
    params_.float3 = req.float3;
    params_.float4 = req.float4;
    params_.float5 = req.float5;
    params_.float6 = req.float6;
    params_.float7 = req.float7;
    params_.int1 = req.int1;
    params_.bool1 = req.bool1;
    params_.bool2 = req.bool2;
    params_.bool3 = req.bool3;
    params_.bool4 = req.bool4;
    params_.bool5 = req.bool5;
    params_.bool6 = req.bool6;
    params_.bool7 = req.bool7;
    params_.bool8 = req.bool8;
    params_.procType = static_cast<PROC_TYPES_T>(req.procType);
    params_.serialNum = req.serialNum;
    //ROS_INFO("ACTION CALLBACK,\n\t nextActionType = %i,\n\t newActionFlag = %i,\n\t pushToFrontFlag = %i,\n\t clearDequeFlag = %i,\n\t clearFrontFlag = %i,\n\t pause = %i,\n\t float1 = %f,\n\t float2 = %f",
    //         nextActionType_, newActionFlag_, pushToFrontFlag_, clearDequeFlag_, clearFrontFlag_, pause_, params_.float1,params_.float2);
    return true;
}

bool Exec::manualOverrideCallback_(messages::ExecManualOverride::Request &req, messages::ExecManualOverride::Response &res)
{
    manualOverride_ = req.manualOverride;
    return true;
}

void Exec::poseCallback_(const messages::RobotPose::ConstPtr& msg)
{
    robotStatus.xPos = msg->x;
    robotStatus.yPos = msg->y;
	robotStatus.heading = msg->heading;
    robotStatus.bearing = RAD2DEG*atan2(msg->y, msg->x);
}

void Exec::navCallback_(const messages::NavFilterOut::ConstPtr &msg)
{
    robotStatus.yawRate = msg->yaw_rate;
    robotStatus.rollAngle = msg->roll;
    robotStatus.pitchAngle = msg->pitch;
    robotStatus.velocity = msg->velocity;
}

void Exec::grabberCallback_(const messages::GrabberFeedback::ConstPtr& msg)
{
	robotStatus.grabberSlideStatus = msg->slideStatus;
	robotStatus.grabberDropStatus = msg->dropStatus;
	robotStatus.grabberSlidePos = msg->sliderPosAvg;
	robotStatus.grabberDropPos = msg->dropPos;
}

void Exec::driveSpeedsCallback_(const robot_control::DriveSpeeds::ConstPtr &msg)
{
    robotStatus.vMax = msg->vMax;
    robotStatus.rMax = msg->rMax;
}

void Exec::leftRoboteqCallback_(const messages::encoder_data::ConstPtr &msg)
{
    robotStatus.flEncoder = msg->motor_1_encoder_count;
    robotStatus.mlEncoder = msg->motor_2_encoder_count;
    robotStatus.blEncoder = msg->motor_3_encoder_count;
}

void Exec::rightRoboteqCallback_(const messages::encoder_data::ConstPtr &msg)
{
    robotStatus.frEncoder = msg->motor_1_encoder_count;
    robotStatus.mrEncoder = msg->motor_2_encoder_count;
    robotStatus.brEncoder = msg->motor_3_encoder_count;
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
        execInfoMsgOut_.actionBool8[i] = 0;
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
        execInfoMsgOut_.actionBool8[i] = actionDeque_.at(i)->params.bool8;
        execInfoMsgOut_.actionProcType[i] = static_cast<uint8_t>(actionDeque_.at(i)->params.procType);
        execInfoMsgOut_.actionSerialNum[i] = actionDeque_.at(i)->params.serialNum;
	}
}

void Exec::packNextWaypointOut_()
{
    if(actionDeque_.size()>0)
    {
        if(actionDeque_.at(0)->params.actionType == _driveGlobal || actionDeque_.at(0)->params.actionType == _driveRelative)
        {
            nextWaypointMsgOut_.globalX = actionDeque_.at(0)->nextGlobalX;
            nextWaypointMsgOut_.globalY = actionDeque_.at(0)->nextGlobalY;
            //nextWaypointMsgOut_.unskippable = actionDeque_.at(0)->params.bool3;
        }
    }
}

void Exec::packGabberStatusOut_()
{
    grabberStatusMsgOut_.dropFailed = dropFailed_;
    grabberStatusMsgOut_.slidesFailed = slidesFailed_;
}
