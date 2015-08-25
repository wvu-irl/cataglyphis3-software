#include <robot_control/exec_state_machine.h>

ExecStateMachine::ExecStateMachine()
{
    // "allocate" queue memory. but std::queue does't provide an interface for this
    for(int i=0; i<NUM_ACTIONS; i++)
    {
        actionPoolIndex_[i] = 0;
    }
    /*for(int j=0; j<ACTION_POOL_SIZE; j++)
    {
        actionPool_[_idle][j] = new Idle;
        actionPool_[_driveGlobal][j] = new DriveGlobal;

    }*/
    currentAcion_ = actionPoolIndex_[_idle][0];
}

void ExecStateMachine::run()
{
    actionQueueSize_ = actionQueue_.size();
    if(clearQueueFlag_) for(int i=0; i<actionQueueSize_; i++) actionQueue_.pop(); // Clear the queue
    if(newActionFlag_) // New action added to queue
    {
        actionQueue_.push(actionPool_[nextActionType_][actionPoolIndex_[nextActionType_]]);
        actionPoolIndex_[nextActionType_]++;
        if(actionPoolIndex_[nextActionType_]>=ACTION_POOL_SIZE) actionPoolIndex_[nextActionType_] = 0;
    }
    if(pause_) pauseIdle.run();
    else
    {
        if(actionQueue_.empty()) pauseIdle.run();
        else
        {
            if(currentActionDone_)
            {
                currentAction_ = actionQueue_.front();
                actionQueue_.pop();
                currentAction_->init();
            }
            currentAction_->run();
        }
        clearQueueFlag_ = false;
        newActionFlag_ = false;
    }
}
