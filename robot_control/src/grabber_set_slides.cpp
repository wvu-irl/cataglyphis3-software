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
}

int GrabberSetSlides::run()
{
    switch(state_)
    {
    case _normal_:
        robotOutputs.slidePosCmd = slidePos_;
        robotOutputs.grabberStopCmd = 0;
        if(slidesEnded_ && abs(robotStatus.grabberSlidePos - slidePos_) <= slideTol_) {returnValue_ = 1; state_ = _normal_;}
        else if(slidesEnded_ && abs(robotStatus.grabberSlidePos - slidePos_) > slideTol_) {returnValue_ = 0; slidesEnded_ = false; state_ = _recovering_;}
        else {returnValue_ = 0; state_ = _normal_;}
        break;
    case _recovering_:
        if(slidePos_== GRABBER_CLOSED) robotOutputs.slidePosCmd = GRABBER_OPEN;
        else robotOutputs.slidePosCmd = GRABBER_CLOSED;
        robotOutputs.grabberStopCmd = 0;
        if(slidesEnded_) {returnValue_ = 0; numFailedAttempts_++; state_ = _normal_;}
        else {returnValue_ = 0; state_ = _recovering_;}
        if(limitedRetry_ && (numFailedAttempts_ >= maxFailedAttempts_)) {slidesFailed_ = true; returnValue_ = 1;}
        break;
    }
    return returnValue_;
}
