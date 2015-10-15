#include <robot_control/drive_straight.h>

void DriveStraight::init()
{
	initX_ = robotStatus.xPos;
	initY_ = robotStatus.yPos;
	initHeading_ = robotStatus.heading;
	desiredDistance_ = params.float1;
	vMax_ = params.float2;
	if(desiredDistance_<0.0) driveSign_ = -1;
	else driveSign_ = 1;
	timeoutValue_ = 30.0 + 1.0*fabs(desiredDistance_);
	initTime_ = ros::Time::now().toSec();
	headingErrorSpeedI_ = 0.0;
}

int DriveStraight::run()
{
	traversedDistance_ = driveSign_*hypot(robotStatus.xPos-initX_,robotStatus.yPos-initY_);
	remainingDistance_ = desiredDistance_ - traversedDistance_;
	vDesRaw_ = kpV_*remainingDistance_;
	if(vDesRaw_>0.0) vDesCoerc_ = vDesRaw_+vMin_;
	else if(vDesRaw_<0.0) vDesCoerc_ = vDesRaw_-vMin_;
	else vDesCoerc_ = vDesRaw_;
	if(vDesCoerc_>vMax_) vDesCoerc_ = vMax_;
	else if(vDesCoerc_<(-vMax_)) vDesCoerc_ = -vMax_;
	deltaHeading_ = initHeading_ - robotStatus.heading;
	rDes_ = kpR_*deltaHeading_;
	if(rDes_>rMax_) rDes_ = rMax_;
	else if(rDes_<(-rMax_)) rDes_ = -rMax_;
	errorR_ = rDes_ - robotStatus.yawRate;
	headingErrorSpeedP_ = kpR_*rDes_;
	headingErrorSpeedI_ = kiR_*errorR_;
	if(headingErrorSpeedI_>maxHeadingErrorSpeedI_) headingErrorSpeedI_ = maxHeadingErrorSpeedI_;
	else if(headingErrorSpeedI_<(-maxHeadingErrorSpeedI_)) headingErrorSpeedI_ = -maxHeadingErrorSpeedI_;
	headingErrorSpeedT_ = headingErrorSpeedP_ + headingErrorSpeedI_;
	leftSpeed_ = round(kVOutput_*vDesCoerc_+headingErrorSpeedT_);
	rightSpeed_ = round(kVOutput_*vDesCoerc_-headingErrorSpeedT_);
	elapsedTime_ = ros::Time::now().toSec() - initTime_;
	if(fabs(traversedDistance_) >= fabs(desiredDistance_) || elapsedTime_ >= timeoutValue_)
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
		robotOutputs.mlMotorSpeed = leftSpeed_;
		robotOutputs.blMotorSpeed = leftSpeed_;
		robotOutputs.frMotorSpeed = rightSpeed_;
		robotOutputs.mrMotorSpeed = rightSpeed_;
		robotOutputs.brMotorSpeed = rightSpeed_;
		taskEnded_ = 0;
	}
	return taskEnded_;
}
