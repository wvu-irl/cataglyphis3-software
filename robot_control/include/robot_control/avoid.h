#ifndef AVOID_H
#define AVOID_H
#include "procedure.h"
#include "action_type_enum.h"

class Avoid : public Procedure
{
public:
	// Members
	bool interruptedEmergencyEscape;
	int interruptedAvoid;
	// Methods
	Avoid();
	bool runProc();
};

#endif // AVOID_H
