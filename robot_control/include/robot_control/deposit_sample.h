#ifndef DEPOSIT_SAMPLE_H
#define DEPOSIT_SAMPLE_H
#include "procedure.h"

class DepositSample : public Procedure
{
public:
	// Members
	bool dropFailed;
	// Methods
	bool runProc();
};

#endif // DEPOSIT_SAMPLE_H
