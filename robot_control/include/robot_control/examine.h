#ifndef EXAMINE_H
#define EXAMINE_H
#include "procedure.h"

class Examine : public Procedure
{
public:
	// Members
	const int examineLimit = 2;
	const float examineOffsetAngle = 10.0; // deg
	const float maxDistanceToDrive = 5.0; // m
	float finalAngleToTurn;
	// Methods
	bool runProc();
};

#endif // EXAMINE_H
