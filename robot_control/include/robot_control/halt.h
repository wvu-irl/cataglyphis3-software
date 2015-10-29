#ifndef HALT_H
#define HALT_H
#include "action.h"

class Halt : public Action
{
public:
	void init();
	int run();
};

#endif // HALT_H
