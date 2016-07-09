#include <robot_control/drive_halt.h>

void DriveHalt::init()
{
	robotOutputs.flMotorSpeed = 0;
	robotOutputs.mlMotorSpeed = 0;
	robotOutputs.blMotorSpeed = 0;
	robotOutputs.frMotorSpeed = 0;
	robotOutputs.mrMotorSpeed = 0;
	robotOutputs.brMotorSpeed = 0;
    robotOutputs.stopFlag = false;
    robotOutputs.turnFlag = false;
    stopCounts_ = 0;
    speedI_ = 0.0;
    posError_ = 0.0;
    vPrev_ = robotStatus.velocity;
    prevTime_ = ros::Time::now().toSec();
    if(hypot(robotStatus.rollAngle,robotStatus.pitchAngle)>minTiltForHold_) state_ = _waitingForStop;
    else state_ = _noHold;
}

int DriveHalt::run()
{
    switch(state_)
    {
    case _noHold:
        robotOutputs.flMotorSpeed = 0;
        robotOutputs.mlMotorSpeed = 0;
        robotOutputs.blMotorSpeed = 0;
        robotOutputs.frMotorSpeed = 0;
        robotOutputs.mrMotorSpeed = 0;
        robotOutputs.brMotorSpeed = 0;
        robotOutputs.stopFlag = true;
        robotOutputs.turnFlag = false;
        state_ = _noHold;
        break;
    case _waitingForStop:
        robotOutputs.flMotorSpeed = 0;
        robotOutputs.mlMotorSpeed = 0;
        robotOutputs.blMotorSpeed = 0;
        robotOutputs.frMotorSpeed = 0;
        robotOutputs.mrMotorSpeed = 0;
        robotOutputs.brMotorSpeed = 0;
        robotOutputs.stopFlag = false;
        robotOutputs.turnFlag = false;
        if(fabs(robotStatus.velocity)<=stopVelocityThreshold_) stopCounts_++;
        else stopCounts_ = 0;
        if(stopCounts_>=stopCountsThreshold_) state_ = _holding;
        else state_ = _waitingForStop;
        break;
    case _holding:
        if(fabs(posError_>=maxErrorThreshold_))
        {
            posError_ = 0.0;
            speedI_ = 0.0;
        }
        vCurrent_ = robotStatus.velocity;
        if(vCurrent_>vLimit_) vCurrent_ = vLimit_;
        else if(vCurrent_<-vLimit_) vCurrent_ = -vLimit_;
        posError_ += (vCurrent_+vPrev_)/2.0*(ros::Time::now().toSec()-prevTime_);
        vDes_ = -kV_*posError_;
        speedP_ = kpSpeed_*vDes_;
        speedI_ -= kiSpeed_*posError_;
        if(speedI_>speedIMax_) speedI_ = speedIMax_;
        else if(speedI_<-speedIMax_) speedI_ = -speedIMax_;
        speedT_ = speedP_ + speedI_;
        if(speedT_>speedTMax_) speedT_ = speedTMax_;
        else if(speedT_<-speedTMax_) speedT_ = -speedTMax_;
        speedOut_ = round(speedT_);
        robotOutputs.flMotorSpeed = speedOut_;
        robotOutputs.frMotorSpeed = speedOut_;
        robotOutputs.mlMotorSpeed = speedOut_;
        robotOutputs.mrMotorSpeed = speedOut_;
        robotOutputs.blMotorSpeed = speedOut_;
        robotOutputs.brMotorSpeed = speedOut_;
        robotOutputs.stopFlag = false;
        robotOutputs.turnFlag = false;
        vPrev_ = robotStatus.velocity;
        prevTime_ = ros::Time::now().toSec();
        state_ = _holding;
        break;
    }
	return 1;
}
