#include <robot_control/drive_straight.h>

void DriveStraight::init()
{
	initX_ = robotStatus.xPos;
	initY_ = robotStatus.yPos;
	initHeading_ = robotStatus.heading;
	desiredDistance_ = params.float1;
	if(desiredDistance_<0.0) driveSign_ = -1;
	else driveSign_ = 1;
	timeoutValue_ = (unsigned int)round((30.0 + 1.0*fabs(desiredDistance_))*robotStatus.loopRate);
	timeoutCounter_ = 0;
	headingErrorSpeedI_ = 0.0;
    robotOutputs.stopFlag = false;
    robotOutputs.turnFlag = false;
}

int DriveStraight::run()
{
    //int temp;
    //ROS_INFO_THROTTLE(1, "drive straight running");
    //ROS_INFO("============================");
    robotOutputs.stopFlag = false;
    robotOutputs.turnFlag = false;
    vMax_ = robotStatus.vMax;
	traversedDistance_ = driveSign_*hypot(robotStatus.xPos-initX_,robotStatus.yPos-initY_);
	remainingDistance_ = desiredDistance_ - traversedDistance_;
    //ROS_INFO("desiredDistance = %f",desiredDistance_);
    //ROS_INFO("remainingDistance = %f",remainingDistance_);
    //ROS_INFO("traversedDistance = %f",traversedDistance_);
    //ROS_INFO("stopFlag = %i",robotOutputs.stopFlag);
    //ROS_INFO("turnFlag = %i",robotOutputs.turnFlag);
	vDesRaw_ = kpV_*remainingDistance_;
    //ROS_INFO("vDesRaw_ = %f",vDesRaw_);
	if(vDesRaw_>0.0) vDesCoerc_ = vDesRaw_+vMin_;
	else if(vDesRaw_<0.0) vDesCoerc_ = vDesRaw_-vMin_;
	else vDesCoerc_ = vDesRaw_;
	if(vDesCoerc_>vMax_) vDesCoerc_ = vMax_;
	else if(vDesCoerc_<(-vMax_)) vDesCoerc_ = -vMax_;
	deltaHeading_ = initHeading_ - robotStatus.heading;
    //ROS_INFO("deltaHeading_ = %f",deltaHeading_);
	rDes_ = kpR_*deltaHeading_;
	if(rDes_>rMax_) rDes_ = rMax_;
	else if(rDes_<(-rMax_)) rDes_ = -rMax_;
    //ROS_INFO("rDes_ = %f",rDes_);
    if(std::isnan(robotStatus.yawRate)) {ROS_ERROR("yaw rate is nan"); robotStatus.yawRate = yawRatePrev_;}
    else yawRatePrev_ = robotStatus.yawRate;
	errorR_ = rDes_ - robotStatus.yawRate;
    //ROS_INFO("yawRate = %f",robotStatus.yawRate);
    //if(std::isnan(robotStatus.yawRate)) {ROS_ERROR("yaw rate is nan"); std::cin >> temp;}
    //ROS_INFO("errorR_ = %f",errorR_);
	headingErrorSpeedP_ = kpR_*rDes_;
    //ROS_INFO("headingErrorSpeedP_ = %f",headingErrorSpeedP_);
    headingErrorSpeedI_ += kiR_*errorR_;
    //ROS_INFO("headingErrorSpeedI_ before coerc = %f",headingErrorSpeedI_);
	if(headingErrorSpeedI_>maxHeadingErrorSpeedI_) headingErrorSpeedI_ = maxHeadingErrorSpeedI_;
	else if(headingErrorSpeedI_<(-maxHeadingErrorSpeedI_)) headingErrorSpeedI_ = -maxHeadingErrorSpeedI_;
    //ROS_INFO("headingErrorSpeedI_ after coerc = %f",headingErrorSpeedI_);
    headingErrorSpeedT_ = headingErrorSpeedP_ + headingErrorSpeedI_;
    //ROS_INFO("headingErrorSpeedT_ before coerc = %f",headingErrorSpeedT_);
    if(headingErrorSpeedT_>maxHeadingErrorSpeed_) headingErrorSpeedT_ = maxHeadingErrorSpeed_;
    else if(headingErrorSpeedT_<(-maxHeadingErrorSpeed_)) headingErrorSpeedT_ = -maxHeadingErrorSpeed_;
    //ROS_INFO("headingErrorSpeedT_ after coerc = %f",headingErrorSpeedT_);
	leftSpeed_ = (int)round(kVOutput_*vDesCoerc_+headingErrorSpeedT_);
	rightSpeed_ = (int)round(kVOutput_*vDesCoerc_-headingErrorSpeedT_);
    //ROS_INFO("vDesCoerc_ = %f",vDesCoerc_);
    //ROS_INFO("leftSpeed_ = %i",leftSpeed_);
    //ROS_INFO("rightSpeed_ = %i",rightSpeed_);
	timeoutCounter_++;
	if(fabs(traversedDistance_) >= fabs(desiredDistance_) || timeoutCounter_ >= timeoutValue_)
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
    //ROS_INFO("task ended = %i",taskEnded_);
    //std::printf("\n");
	return taskEnded_;
}
