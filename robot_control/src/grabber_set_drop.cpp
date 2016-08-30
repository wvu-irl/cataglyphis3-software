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
}

int GrabberSetDrop::run()
{
    switch(state_)
    {
    case _normal_:
        robotOutputs.dropPosCmd = dropPos_;
        robotOutputs.grabberStopCmd = 0;
        if(dropEnded_ && abs(robotStatus.grabberDropPos - dropPos_) <= dropTol_) {returnValue_ = 1; state_ = _normal_;}
        else if(dropEnded_ && abs(robotStatus.grabberDropPos - dropPos_) > dropTol_) {returnValue_ = 0; state_ = _recovering_;}
        else {returnValue_ = 0; state_ = _normal_;}
        break;
    case _recovering_:
        if(dropPos_ == GRABBER_DROPPED) robotOutputs.dropPosCmd = GRABBER_RAISED;
        else robotOutputs.dropPosCmd = GRABBER_DROPPED;
        robotOutputs.grabberStopCmd = 0;
        if(dropEnded_) {returnValue_ = 0; numFailedAttempts_++; state_ = _normal_;}
        else {returnValue_ = 0; state_ = _recovering_;}
        if(limitedRetry_ && (numFailedAttempts_ >= maxFailedAttempts_)) {dropFailed_ = true; returnValue_ = 1;}
        break;
    }
    return returnValue_;
}
