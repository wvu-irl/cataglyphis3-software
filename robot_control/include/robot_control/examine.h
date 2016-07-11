#ifndef EXAMINE_H
#define EXAMINE_H
#include "procedure.h"

class Examine : public Procedure
{
public:
	// Members
	const int examineLimit = 2;
	const float offsetPositionAngle = 90.0; // deg
	const float offsetPositionDistance = 3.0; // m
	// Methods
	bool runProc();
};

#endif // EXAMINE_H
