#ifndef SOS_MODE_H
#define SOS_MODE_H
#include "procedure.h"

class SosMode : public Procedure
{
public:
	// Members
	float turnAngle;
	const float maxTurnAngle = 90.0; // deg
	// Methods
	SosMode();
	bool runProc();
};

#endif // SOS_MODE_H
