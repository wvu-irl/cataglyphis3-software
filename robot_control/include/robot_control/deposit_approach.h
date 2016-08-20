#ifndef DEPOSIT_APPROACH_H
#define DEPOSIT_APPROACH_H
#include "procedure.h"

enum DEPOSIT_APPROACH_STEP_T {_biasRemoval ,_align, _platform};

class DepositApproach : public Procedure
{
public:
	// Members
	std::vector<robot_control::Waypoint> depositLocations;
	const float depositAlignXDistance = 3.0; // m
	DEPOSIT_APPROACH_STEP_T step;
	float platformDriveDistance;
	float platformPivotAngle;
	// Methods
	DepositApproach(); // constructor
	bool runProc();
	void calcPlatformDrive();
};

#endif // DEPOSIT_APPROACH_H
