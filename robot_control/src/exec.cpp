#include <robot_control/exec.h>

Exec::Exec()
{
	pauseIdle_.robotStatus.loopRate = loopRate;
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
	}
	for(int k=0; k<NUM_TASKS; k++)
	{
		pauseIdle_.taskPoolIndex[k] = 0;
	}
	for(int l=0; l<ACTION_POOL_SIZE; l++)
	{
		pauseIdle_.taskPool[_driveHalt_][l] = new DriveHalt;
		pauseIdle_.taskPool[_driveStraight_][l] = new DriveStraight;
		pauseIdle_.taskPool[_driveStraightCL_][l] = new DriveStraightCL;
		pauseIdle_.taskPool[_grabberHalt_][l] = new GrabberHalt;
		pauseIdle_.taskPool[_grabberIdle_][l] = new GrabberIdle;
		pauseIdle_.taskPool[_grabberSetDrop_][l] = new GrabberSetDrop;
		pauseIdle_.taskPool[_grabberSetSlides_][l] = new GrabberSetSlides;
		pauseIdle_.taskPool[_visionHalt_][l] = new VisionHalt;
		// remaining vision tasks
	}

}

void Exec::run()
{
    actionDequeSize_ = actionDeque_.size(); // Record deque size
    if(clearDequeFlag_) actionDeque_.clear(); // Clear deque
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
            if(newActionFlag_ && !pushToFrontFlag_) actionDeque_.front()->init(); // If a new action has been pushed to the back of the deque, run init() on the new action
        }
        else // Else, deque is not empty and the action at the front() should be run
        {
            if(pushToFrontFlag_) actionDeque_.front()->init(); // If new action was pushed to the front, need to run init() for new action
            if(currentActionDone_) // If the action at the front of the deque is complete, pop this action off and init() the next action in the deque
            {
                actionDeque_.pop_front();
                actionDeque_.front()->init();
            }
            currentActionDone_ = actionDeque_.front()->run();
        }
    }
    clearDequeFlag_ = false;
    newActionFlag_ = false;
    pushToFrontFlag_ = false;
	pausePrev_ = pause_;
}

void Exec::actionCallback_(const messages::ExecAction::ConstPtr& msg)
{
    nextActionType_ = static_cast<ACTION_TYPE_T>(msg->nextActionType);
    pushToFrontFlag_ = msg->pushToFrontFlag;
    clearDequeFlag_ = msg->clearDequeFlag;
    pause_ = msg->pause;
    params_.float1 = msg->float1;
    params_.float2 = msg->float2;
    params_.float3 = msg->float3;
    params_.int1 = msg->int1;
    params_.bool1 = msg->bool1;
}
