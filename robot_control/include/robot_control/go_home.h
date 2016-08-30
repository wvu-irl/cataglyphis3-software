#ifndef GO_HOME_H
#define GO_HOME_H
#include "procedure.h"

class GoHome : public Procedure
{
public:
	// Members
	const float goHomeDistanceTolerance = 1.5; // m
	// Methods
	bool runProc();
};

#endif // GO_HOME_H
