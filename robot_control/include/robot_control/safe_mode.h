#ifndef SAFE_MODE_H
#define SAFE_MODE_H
#include "procedure.h"

class SafeMode : public Procedure
{
public:
	// Members
	float driveDistance;
	float turnAngle;
	const float maxDriveDistance = 2.0; // m
	const float minDriveDistance = 0.5; // m
	const float maxTurnAngle = 90.0; // deg
	// Methods
	SafeMode();
	bool runProc();
};

#endif // SAFE_MODE_H
