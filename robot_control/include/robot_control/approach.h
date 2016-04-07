#ifndef APPROACH_H
#define APPROACH_H
#include "procedure.h"

enum APPROACH_STEP_T {_computeManeuver, _performManeuver};

class Approach : public Procedure
{
public:
	// Members
	APPROACH_STEP_T step;
	uint8_t sampleTypeMux;
	const float maxDriveDistance = 4.0; // m
	int backUpCount = 0;
	const int maxBackUpCount = 1;
	const float backUpDistance = -0.5;
	const int approachValueThreshold = 700;
	bool commandedSearch;
	bool approachableSample;
	// Methods
	Approach(); // constructor
	bool runProc();
};

#endif // APPROACH_H
