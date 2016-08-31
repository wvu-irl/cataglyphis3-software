#include <robot_control/grabber_set_slides.h>

void GrabberSetSlides::init()
{
    slidePos_ = params.int1;
    if(slidePos_ == limitedRetryPosition_) limitedRetry_ = true;
    else limitedRetry_ = false;
    state_ = _normal_;
    numFailedAttempts_ = 0;
    slidesFailed_ = false;
    slidesEnded_ = false;
    slidesFinishedCount_ = 0;
    slidesStuckOpenWaitCount_ = 0;
}

int GrabberSetSlides::run()
{
    if(robotStatus.grabberSlideStatus==1) slidesFinishedCount_++;
    else slidesFinishedCount_ = 0;
    if(slidesFinishedCount_>maxSlidesFinishedCount_) state_ = _stuckOpenWait_;
    switch(state_)
    {
    case _normal_:
        //ROS_INFO("grabber set slides normal state");
        robotOutputs.slidePosCmd = slidePos_;
        robotOutputs.grabberStopCmd = 0;
        if(slidesEnded_ && abs(robotStatus.grabberSlidePos - slidePos_) <= slideTol_) {returnValue_ = 1; state_ = _normal_;}
        //else if(slidesEnded_ && abs(robotStatus.grabberSlidePos - slidePos_) > slideTol_) {returnValue_ = 0; slidesEnded_ = false; state_ = _recovering_;}
        else {returnValue_ = 0; state_ = _normal_;}
        break;
    case _recovering_:
        //ROS_INFO("grabber set slides recovering state");
        if(slidePos_== GRABBER_CLOSED) slideRecoveryPos_ = GRABBER_OPEN;
        else slideRecoveryPos_ = GRABBER_CLOSED;
        robotOutputs.dropPosCmd = slideRecoveryPos_;
        robotOutputs.grabberStopCmd = 0;
        if(slidesEnded_) {returnValue_ = 0; numFailedAttempts_++; state_ = _normal_;}
        else {returnValue_ = 0; state_ = _recovering_;}
        if(limitedRetry_ && (numFailedAttempts_ >= maxFailedAttempts_)) {slidesFailed_ = true; returnValue_ = 1;}
        break;
    case _stuckOpenWait_:
        ROS_INFO("grabber stuck, waiting to open");
        robotOutputs.slidePosCmd = GRABBER_OPEN;
        robotOutputs.grabberStopCmd = 0;
        slidesStuckOpenWaitCount_++;
        slidesFailed_ = true;
        if(slidesStuckOpenWaitCount_ > maxSlidesStuckOpenWaitCount_) returnValue_ = 1;
        else returnValue_ = 0;
        break;
    }
    return returnValue_;
}
