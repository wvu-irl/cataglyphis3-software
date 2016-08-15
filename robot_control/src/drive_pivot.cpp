#include <robot_control/drive_pivot.h>

void DrivePivot::init()
{
	initHeading_ = robotStatus.heading;
	desiredDeltaHeading_ = params.float1;
	if(deltaHeading_<0.0) pivotSign_ = -1;
	else pivotSign_ = 1;
    timeoutValue_ = (unsigned int)round((10.0 + fabs(desiredDeltaHeading_)/10.0)*robotStatus.loopRate);
	timeoutCounter_ = 0;
	rSpeedI_ = 0.0;
	inThreshold_ = false;
	thresholdTime_ = 0.0;
    robotOutputs.stopFlag = false;
    robotOutputs.turnFlag = true;
    flEncPrev_ = robotStatus.flEncoder;
    mlEncPrev_ = robotStatus.mlEncoder;
    blEncPrev_ = robotStatus.blEncoder;
    frEncPrev_ = robotStatus.frEncoder;
    mrEncPrev_ = robotStatus.mrEncoder;
    brEncPrev_ = robotStatus.brEncoder;
    dogLegState = _monitoring;
    dogLegDetectTimeStarted_ = false;
    ROS_DEBUG("drivePivot init");
}

int DrivePivot::run()
{
    robotOutputs.stopFlag = false;
    robotOutputs.turnFlag = true;
    rMax_ = robotStatus.rMax;
	deltaHeading_ = robotStatus.heading - initHeading_;
	rDes_ = kpR_*(desiredDeltaHeading_-deltaHeading_);
    dogLeg_();
	if(rDes_>rMax_) rDes_ = rMax_;
	else if(rDes_<(-rMax_)) rDes_ = -rMax_;
    if(rDes_>0.0)
	{
		ccwBoostGain_ = cornerBoostGain_;
		cwBoostGain_ = 1.0;
		rightMiddleWheelMultiplier_ = reverseMiddleGain_;
		leftMiddleWheelMultiplier_ = 1.0;
	}
    else
	{
		ccwBoostGain_ = 1.0;
		cwBoostGain_ = cornerBoostGain_;
		rightMiddleWheelMultiplier_ = 1.0;
		leftMiddleWheelMultiplier_ = reverseMiddleGain_;
	}
	errorR_ = rDes_ - robotStatus.yawRate;
	rSpeedP_ = kROutput_*rDes_;
    rSpeedI_ += kiR_*errorR_;
    if(rSpeedI_>rSpeedIMax_) rSpeedI_ = rSpeedIMax_;
    else if(rSpeedI_<-rSpeedIMax_) rSpeedI_ = -rSpeedIMax_;
    rSpeedT_ = round(rSpeedP_ + rSpeedI_);
	if(rSpeedT_>rSpeedMax_) rSpeedT_ = rSpeedMax_;
	else if(rSpeedT_<(-rSpeedMax_)) rSpeedT_ = -rSpeedMax_;
	ROS_DEBUG("rSpeedT: %i",rSpeedT_);
	ROS_DEBUG("rDes: %f",rDes_);
    ROS_DEBUG("desiredDeltaHeading = %f", desiredDeltaHeading_);
    ROS_DEBUG("deltaHeading = %f", deltaHeading_);
	ROS_DEBUG("desiredDeltaHeading_-deltaHeading_: %f", desiredDeltaHeading_-deltaHeading_);
	leftSpeed_ = rSpeedT_;
	rightSpeed_ = -rSpeedT_;
	timeoutCounter_++;
	if(fabs(desiredDeltaHeading_-deltaHeading_)<=deltaHeadingThreshold_ && inThreshold_==false) {thresholdInitTime_ = ros::Time::now().toSec(); inThreshold_ = true;}
	if(fabs(desiredDeltaHeading_-deltaHeading_)<=deltaHeadingThreshold_) thresholdTime_ = ros::Time::now().toSec() - thresholdInitTime_;
	else {thresholdTime_ = 0.0; inThreshold_ = false;}
	if(thresholdTime_ >= thresholdMinTime_ || timeoutCounter_ >= timeoutValue_)
	{
		robotOutputs.flMotorSpeed = 0;
		robotOutputs.mlMotorSpeed = 0;
		robotOutputs.blMotorSpeed = 0;
		robotOutputs.frMotorSpeed = 0;
		robotOutputs.mrMotorSpeed = 0;
		robotOutputs.brMotorSpeed = 0;
		ROS_DEBUG("end pivot");
		taskEnded_ = 1;
	}
	else
	{
        robotOutputs.flMotorSpeed = (int)round(leftSpeed_*cwBoostGain_);
        robotOutputs.mlMotorSpeed = (int)round(leftSpeed_*middleWheelReduction_*leftMiddleWheelMultiplier_);
        robotOutputs.blMotorSpeed = (int)round(leftSpeed_*ccwBoostGain_);
        robotOutputs.frMotorSpeed = (int)round(rightSpeed_*ccwBoostGain_);
        robotOutputs.mrMotorSpeed = (int)round(rightSpeed_*middleWheelReduction_*rightMiddleWheelMultiplier_);
        robotOutputs.brMotorSpeed = (int)round(rightSpeed_*cwBoostGain_);
		taskEnded_ = 0;
	}
	return taskEnded_;
}

void DrivePivot::dogLeg_()
{
    switch(dogLegState)
    {
    case _monitoring:
        encoderDiffSum_ = abs(robotStatus.flEncoder - flEncPrev_) + abs(robotStatus.mlEncoder - mlEncPrev_) + abs(robotStatus.blEncoder - blEncPrev_) +
                abs(robotStatus.frEncoder - frEncPrev_) + abs(robotStatus.mrEncoder - mrEncPrev_) + abs(robotStatus.brEncoder - brEncPrev_);
        if(encoderDiffSum_ > encoderDogLegTriggerValue_ && !dogLegDetectTimeStarted_) {dogLegDetectTimeStarted_ = true; dogLegDetectTime_ = ros::Time::now().toSec();}
        else if(encoderDiffSum_ <= encoderDogLegTriggerValue_ && dogLegDetectTimeStarted_) dogLegDetectTimeStarted_ = false;
        if(dogLegDetectTimeStarted_ && (ros::Time::now().toSec() - dogLegDetectTime_) >= dogLegDetectThreshold_) dogLegState = _commanding;
        else dogLegState = _monitoring;
        break;
    case _commanding:
        ROS_INFO("dog leg detected");
        rSpeedI_ = 0.0;
        dogLegRecoverStartTime_ = ros::Time::now().toSec();
        dogLegState = _recovering;
        break;
    case _recovering:
        if((ros::Time::now().toSec() - dogLegRecoverStartTime_) >= dogLegRecoverDuration_)
        {
            if(pivotSign_ > 0) rDes_ = -dogLegRDes_;
            else rDes_ = dogLegRDes_;
            dogLegState = _recovering;
        }
        else
        {
            rSpeedI_ = 0.0;
            dogLegDetectTimeStarted_ = false;
            dogLegState = _monitoring;
        }
        break;
    }
    flEncPrev_ = robotStatus.flEncoder;
    mlEncPrev_ = robotStatus.mlEncoder;
    blEncPrev_ = robotStatus.blEncoder;
    frEncPrev_ = robotStatus.frEncoder;
    mrEncPrev_ = robotStatus.mrEncoder;
    brEncPrev_ = robotStatus.brEncoder;
}
