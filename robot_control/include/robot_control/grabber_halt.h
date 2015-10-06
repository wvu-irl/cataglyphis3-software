#ifndef GRABBER_HALT_H
#define GRABBER_HALT_H
#include "task.h"

class GrabberHalt : public Task
{
public:
	void init();
	int run();
};

#endif // GRABBER_HALT_H
