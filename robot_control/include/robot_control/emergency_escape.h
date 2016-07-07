#ifndef EMERGENCY_ESCAPE_H
#define EMERGENCY_ESCAPE_H
#include "procedure.h"
#include "action_type_enum.h"

class EmergencyEscape : public Procedure
{
public:
	// Members
	bool interrupted = false;
	const float backupDistance = -2.0; // m
	const float offsetAngle = 90.0; // deg
	const float offsetDistance = 2.0; // m
	unsigned int backupSerialNum;
	unsigned int offsetDriveSerialNum;
	// Methods
	bool runProc();
};

#endif // EMERGENCY_ESCAPE_H
