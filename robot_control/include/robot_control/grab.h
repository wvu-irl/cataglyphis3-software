#ifndef GRAB_H
#define GRAB_H
#include "action.h"

class Grab : public Action
{
public:
	void init();
	int run();
};

#endif // GRAB_H
