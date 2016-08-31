#ifndef DRIVE_HALT_H
#define DRIVE_HALT_H
#include "task.h"

class DriveHalt : public Task
{
public:
	void init();
	int run();
private:
	enum DRIVE_HALT_STATES_T {_noHold, _waitingForStop, _holding} state_ = _noHold;
	const float minTiltForHold_ = 6.0; // deg
	double dt_;
	double prevTime_;
	float posError_;
	float vCurrent_;
	float vPrev_;
	float vDes_;
	float speedP_;
	float speedI_;
	float speedT_;
	int stopCounts_;
	int speedOut_;
	const float kV_ = 0.5;
	const float kpSpeed_ = 900.0;
	const float kiSpeed_ = 0.15;
	const float vLimit_ = 1.2; // m/s
	const float maxErrorThreshold_ = 0.2; // m
	const float speedIMax_ = 200.0;
	const float speedTMax_ = 300.0;
	const float stopVelocityThreshold_ = 0.01; // m/s
	const int stopCountsThreshold_ = 40; // counts @ 20 Hz
};

#endif // DRIVE_HALT_H
