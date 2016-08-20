#ifndef INITIALIZE_H
#define INITIALIZE_H
#include "procedure.h"

enum INITIALIZE_STEP_T {_drivingOffPlatform, _firstBiasRemoval};

class Initialize : public Procedure
{
public:
	INITIALIZE_STEP_T step;
	bool runProc();
	const float driveOffPlatformDistance = 4.0; // m
};

#endif // INITIALIZE_H