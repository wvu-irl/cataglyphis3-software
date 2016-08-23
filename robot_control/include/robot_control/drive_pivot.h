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
	float yawRatePrev_;
	int pivotSign_;
	float leftSpeed_;
	float rightSpeed_;
	float rSpeedT_;
	float rSpeedP_;
	float rSpeedI_;
	unsigned int timeoutValue_;
	unsigned int timeoutCounter_;
	double thresholdInitTime_;
	double thresholdTime_;
	bool inThreshold_;
	int taskEnded_;
	const float kpR_ = 0.85; // deg/(s*deg)
	const float kiR_ = 0.15;
	const float rSpeedIMax_ = 100.0;
	const float kROutput_ = 450/45.0; // 45% of max speed at 45 deg/s
	const int rSpeedMax_ = 500;
	const float deltaHeadingThreshold_ = 2.0; // deg
	const double thresholdMinTime_ = 0.25; // s
	const float middleWheelReduction_ = 0.65;
	const float cornerBoostGain_ = 1.2;
	const float reverseMiddleGain_ = 0.8;
	float ccwBoostGain_;
	float cwBoostGain_;
	float leftMiddleWheelMultiplier_;
	float rightMiddleWheelMultiplier_;
	void dogLeg_();
	double dogLegDetectTime_;
	double dogLegStopTime_;
	double dogLegRecoverStartTime_;
	const double dogLegDetectThreshold_ = 2.0; // sec
	const double dogLegStopDuration_ = 0.5; // sec
	const double dogLegRecoverDuration_ = 0.75; // sec
	const long int encoderDogLegTriggerValue_ = 100;
	long int encPrev_[6];
	long int encDelta_[6];
	std::vector<long int> leftDeltaVector_;
	std::vector<long int> rightDeltaVector_;
	const size_t rollingAverageSize_ = 20;
	long int leftDeltaAverage_;
	long int rightDeltaAverage_;
	long int maxLeftDelta_;
	long int maxRightDelta_;
	long int minLeftDelta_;
	long int minRightDelta_;
    long int leftMaxMinusMin_;
    long int rightMaxMinusMin_;
    const long int maxMinusMinLimit_ = 300;
	const float dogLegRDes_ = 45.0;
	bool dogLegDetectTimeStarted_;
	enum DOG_LEG_STATE_T {_monitoring, _stoppingFirst, _recovering, _stoppingSecond} dogLegState;
};

#endif // DRIVE_PIVOT_H
