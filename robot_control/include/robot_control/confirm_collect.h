#ifndef CONFIRM_COLLECT_H
#define CONFIRM_COLLECT_H
#include "procedure.h"

class ConfirmCollect : public Procedure
{
public:
	// Members
	uint8_t sampleTypeMux;
	// Methods
	bool runProc();
};

#endif // CONFIRM_COLLECT_H
