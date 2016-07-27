#ifndef SQUARE_UPDATE_H
#define SQUARE_UPDATE_H
#include "procedure.h"

class SquareUpdate : public Procedure
{
public:
	// Members
	const float cornerX = 7.0; // m
	const float cornerY = 7.0; // m
	// Methods
	bool runProc();
};

#endif // SQUARE_UPDATE_H
