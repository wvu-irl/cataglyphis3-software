#ifndef REORIENT_H
#define REORIENT_H
#include "procedure.h"

class Reorient : public Procedure
{
public:
	// Members
	const float reorientPivotAngle = 45.0; // deg
	const float reorientDriveDistance = 3.45; // m
	// Methods
	bool runProc();
};

#endif // REORIENT_H
