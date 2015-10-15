#ifndef DRIVE_PIVOT_H
#define DRIVE_PIVOT_H
#include "task.h"

class DrivePivot : public Task
{
public:
	void init();
	int run();
private:
	float desiredDeltaHeading_;
	float deltaHeading_;
	float rMax_;
	float initHeading_;
	float rDes_;
	float errorR_;
	int pivotSign_;
	int leftSpeed_;
	int rightSpeed_;
	int rSpeedT_;
	float rSpeedP_;
	float rSpeedI_;
	double timeoutValue_;
	double initTime_;
	double elapsedTime_;
	double thresholdInitTime_;
	double thresholdTime_;
	bool inThreshold_;
	int taskEnded_;
	const float kpR_ = 1.0; // deg/(s*deg)
	const float kiR_ = 0.25;
	const float kROutput_ = 450/45.0; // 45% of max speed at 45 deg/s
	const float rSpeedMax_ = 500;
	const float deltaHeadingThreshold_ = 2.0; // deg
	const double thresholdMinTime_ = 0.25; // s
	const float middleWheelReduction_ = 0.65;
};

#endif // DRIVE_PIVOT_H