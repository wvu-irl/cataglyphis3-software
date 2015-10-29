#include <robot_control/drive_pivot.h>

void DrivePivot::init()
{
	initHeading_ = robotStatus.heading;
	desiredDeltaHeading_ = params.float1;
	rMax_ = params.float2;
	if(deltaHeading_<0.0) pivotSign_ = -1;
	else pivotSign_ = 1;
	timeoutValue_ = (unsigned int)round((30.0 + fabs(deltaHeading_)/10.0)*robotStatus.loopRate);
	timeoutCounter_ = 0;
	rSpeedI_ = 0.0;
	inThreshold_ = false;
	thresholdTime_ = 0.0;
}

int DrivePivot::run()
{
	deltaHeading_ = initHeading_ - robotStatus.heading;
	rDes_ = kpR_*deltaHeading_;
	if(rDes_>rMax_) rDes_ = rMax_;
	else if(rDes_<(-rMax_)) rDes_ = -rMax_;
	errorR_ = rDes_ - robotStatus.yawRate;
	rSpeedP_ = kpR_*rDes_;
	rSpeedI_ = kiR_*errorR_;
	rSpeedT_ = rSpeedP_ + rSpeedI_;
	if(rSpeedT_>rSpeedMax_) rSpeedT_ = rSpeedMax_;
	else if(rSpeedT_<(-rSpeedMax_)) rSpeedT_ = -rSpeedMax_;
	leftSpeed_ = round(rSpeedT_);
	rightSpeed_ = round(-rSpeedT_);
	timeoutCounter_++;
	if(fabs(deltaHeading_)<=deltaHeadingThreshold_ && inThreshold_==false) {thresholdInitTime_ = ros::Time::now().toSec(); inThreshold_ = true;}
	if(fabs(deltaHeading_)<=deltaHeadingThreshold_) thresholdTime_ = ros::Time::now().toSec() - thresholdInitTime_;
	else {thresholdTime_ = 0.0; inThreshold_ = false;}
	if(thresholdTime_ >= thresholdMinTime_ || timeoutCounter_ >= timeoutValue_)
	{
		robotOutputs.flMotorSpeed = 0;
		robotOutputs.mlMotorSpeed = 0;
		robotOutputs.blMotorSpeed = 0;
		robotOutputs.frMotorSpeed = 0;
		robotOutputs.mrMotorSpeed = 0;
		robotOutputs.brMotorSpeed = 0;
		taskEnded_ = 1;
	}
	else
	{
		robotOutputs.flMotorSpeed = leftSpeed_;
		robotOutputs.mlMotorSpeed = leftSpeed_*middleWheelReduction_;
		robotOutputs.blMotorSpeed = leftSpeed_;
		robotOutputs.frMotorSpeed = rightSpeed_;
		robotOutputs.mrMotorSpeed = rightSpeed_*middleWheelReduction_;
		robotOutputs.brMotorSpeed = rightSpeed_;
		taskEnded_ = 0;
	}
	return taskEnded_;
}
