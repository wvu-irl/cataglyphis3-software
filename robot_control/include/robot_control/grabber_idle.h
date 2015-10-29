#ifndef GRABBER_IDLE_H
#define GRABBER_IDLE_H
#include "task.h"

class GrabberIdle : public Task
{
public:
	void init();
	int run();
};

#endif // GRABBER_IDLE_H
