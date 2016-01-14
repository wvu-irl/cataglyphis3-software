#include <robot_control/drive_global.h>

void DriveGlobal::init()
{
	desiredX_ = params.float1;
	desiredY_ = params.float2;
	vMax_ = params.float3;
	rMax_ = params.float4;
	desiredEndHeading_ = params.float5;
	endHeading_ = params.bool1;
	ROS_DEBUG("before task deque clears");
	driveDeque.clear();
	grabberDeque.clear();
	visionDeque.clear();
	calculatePath_();
	ROS_DEBUG("before push task");
	pushTask(_pivot_);
	ROS_DEBUG("after push task");
	ROS_DEBUG("drive deque back: %p",driveDeque.back());
	driveDeque.back()->params.float1 = angleToTurn_;
	driveDeque.back()->params.float2 = rMax_;
	ROS_DEBUG("before next push task");
	pushTask(_driveStraight_);
	ROS_DEBUG("after next push task");
	driveDeque.back()->params.float1 = distanceToDrive_;
	driveDeque.back()->params.float2 = vMax_;
	if(endHeading_)
	{
		pushTask(_pivot_);
		driveDeque.back()->params.float1 = endHeading_ - (robotStatus.heading + angleToTurn_);
		driveDeque.back()->params.float2 = rMax_;
	}
}

int DriveGlobal::run()
{
	ROS_DEBUG("before runDeques");
	return runDeques();
}

void DriveGlobal::calculatePath_()
{
	xErr_ = desiredX_-robotStatus.xPos;
	yErr_ = desiredY_-robotStatus.yPos;
	uXDes_ = xErr_/hypot(xErr_,yErr_);
	uYDes_ = yErr_/hypot(xErr_,yErr_);
	uXAct_ = cos(robotStatus.heading*PI/180.0);
	uYAct_ = sin(robotStatus.heading*PI/180.0);
	if(asin(uXAct_*uYDes_-uXDes_*uYAct_)>=0) newHeadingSign_ = 1.0;
	else newHeadingSign_ = -1.0;
	angleToTurn_ = (180.0/PI)*(newHeadingSign_)*acos(uXAct_*uXDes_+uYAct_*uYDes_);
	distanceToDrive_ = hypot(xErr_,yErr_);
}