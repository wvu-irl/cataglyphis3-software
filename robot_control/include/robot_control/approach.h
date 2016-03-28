#ifndef APPROACH_H
#define APPROACH_H
#include "procedure.h"

enum APPROACH_STEP_T {_performSearch, _driveManeuver, _blindDrive};

class Approach : public Procedure
{
public:
	// Members
	float distanceToDrive; // m
	float angleToTurn; // deg
	APPROACH_STEP_T step;
	uint8_t sampleTypeMux;
	const float maxDriveDistance = 4.0; // m
	int backUpCount = 0;
	const int maxBackUpCount = 1;
	const float backUpDistance = -0.5;
	// Methods
	bool runProc();
};

#endif // APPROACH_H