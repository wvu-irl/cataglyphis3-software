#include <robot_control/drive_relative.h>

void DriveRelative::init()
{
	distanceToDrive_ = params.float1;
	angleToTurn_ = params.float2;
    desiredEndHeading_ = params.float3;
	endHeading_ = params.bool1;
    pushedToFront_ = params.bool2;
    clearDeques();
	pushTask(_pivot_);
	driveDeque.back()->params.float1 = angleToTurn_;
	pushTask(_driveStraight_);
	driveDeque.back()->params.float1 = distanceToDrive_;
	if(endHeading_)
	{
		pushTask(_pivot_);
        candidateEndHeadingAngleToTurn_[0] = desiredEndHeading_ - (fmod(robotStatus.heading, 360.0) + angleToTurn_);
        candidateEndHeadingAngleToTurn_[1] = -desiredEndHeading_ - (fmod(robotStatus.heading, 360.0) + angleToTurn_);
        if(fabs(candidateEndHeadingAngleToTurn_[0]) < fabs(candidateEndHeadingAngleToTurn_[1]))
            driveDeque.back()->params.float1 = candidateEndHeadingAngleToTurn_[0];
        else
            driveDeque.back()->params.float1 = candidateEndHeadingAngleToTurn_[1];
    }
    if(pushedToFront_) initDequesFront();
}

int DriveRelative::run()
{
	return runDeques();
}
