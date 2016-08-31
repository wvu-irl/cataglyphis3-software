#include <robot_control/grabber_set_drop.h>

void GrabberSetDrop::init()
{
	dropPos_ = params.int1;
    if(dropPos_ == limitedRetryPosition_) limitedRetry_ = true;
    else limitedRetry_ = false;
    state_ = _normal_;
    numFailedAttempts_ = 0;
    dropFailed_ = false;
    dropEnded_ = false;
    dropFinishedCount_ = 0;
}

int GrabberSetDrop::run()
{
    switch(state_)
    {
    case _normal_:
        ROS_INFO("grabber set drop normal state");
        robotOutputs.dropPosCmd = dropPos_;
        robotOutputs.grabberStopCmd = 0;
        if(dropEnded_ && abs(robotStatus.grabberDropPos - dropPos_) <= dropTol_) {returnValue_ = 1; state_ = _normal_;}
        //else if(dropEnded_ && abs(robotStatus.grabberDropPos - dropPos_) > dropTol_) {returnValue_ = 0; dropEnded_ = false; state_ = _recovering_;}
        else {returnValue_ = 0; state_ = _normal_;}
        break;
    case _recovering_:
        ROS_INFO("grabber set drop recovering state");
        if(dropPos_ == GRABBER_DROPPED) dropRecoveryPos_ = GRABBER_RAISED;
        else dropRecoveryPos_ = GRABBER_DROPPED;
        robotOutputs.dropPosCmd = dropRecoveryPos_;
        robotOutputs.grabberStopCmd = 0;
        if(dropEnded_) {returnValue_ = 0; numFailedAttempts_++; state_ = _normal_;}
        else {returnValue_ = 0; state_ = _recovering_;}
        if(limitedRetry_ && (numFailedAttempts_ >= maxFailedAttempts_)) {dropFailed_ = true; returnValue_ = 1;}
        break;
    }
    return returnValue_;
}
