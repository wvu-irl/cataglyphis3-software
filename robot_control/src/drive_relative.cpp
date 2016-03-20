#include <robot_control/drive_relative.h>

void DriveRelative::init()
{
	distanceToDrive_ = params.float1;
	angleToTurn_ = params.float2;
	vMax_ = params.float3;
	rMax_ = params.float4;
	desiredEndHeading_ = params.float5;
	endHeading_ = params.bool1;
    clearDeques();
	pushTask(_pivot_);
	driveDeque.back()->params.float1 = angleToTurn_;
	driveDeque.back()->params.float2 = rMax_;
	pushTask(_driveStraight_);
	driveDeque.back()->params.float1 = distanceToDrive_;
	driveDeque.back()->params.float2 = vMax_;
	if(endHeading_)
	{
		pushTask(_pivot_);
		driveDeque.back()->params.float1 = endHeading_ - (robotStatus.heading + angleToTurn_);
		driveDeque.back()->params.float2 = rMax_;
	}
}

int DriveRelative::run()
{
	return runDeques();
}
