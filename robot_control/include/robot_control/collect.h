#ifndef COLLECT_H
#define COLLECT_H
#include "procedure.h"

class Collect : public Procedure
{
public:
	// Members
	bool dropOrSlidesFailed;
	const float failedBackUpDistance = -1.0; // m
	// Methods
	bool runProc();
};

#endif // COLLECT_H
