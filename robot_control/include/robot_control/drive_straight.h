#ifndef DRIVE_STRAIGHT_H
#define DRIVE_STRAIGHT_H
#include "task.h"

class DriveStraight : public Task
{
public:
	void init();
	int run();
private:
	float initX_;
	float initY_;
	float initHeading_;
	int driveSign_;
	int pivotSign_;
	float desiredDistance_;
	float remainingDistance_;
	float traversedDistance_;
	float deltaHeading_;
	float vMax_;
	float vDesRaw_;
	float vDesCoerc_;
	float rDes_;
	float errorR_;
	int leftSpeed_;
	int rightSpeed_;
	int headingErrorSpeedT_;
	float headingErrorSpeedP_;
	float headingErrorSpeedI_;
	double timeoutValue_;
	double initTime_;
	double elapsedTime_;
	int taskEnded_;
	const float vMin_ = 0.03; // m/s
	const float kpV_ = 1.2; // m/(s*m)
	const float kVOutput_ = 900/1.5; // 90% of max speed at 1.5 m/s
	const float kpR_ = 1.0; // deg/(s*deg)
	const float kiR_ = 0.25;
	const float kROutput_ = 450/45.0; // 45% of max speed at 45 deg/s
	const float rMax_ = 30.0; // deg/s
	const int maxHeadingErrorSpeed_ = 100;
	const float maxHeadingErrorSpeedI_ = 50;
};

#endif // DRIVE_STRAIGHT_H
