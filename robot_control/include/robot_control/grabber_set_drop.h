#ifndef GRABBER_SET_DROP_H
#define GRABBER_SET_DROP_H
#include "task.h"

class GrabberSetDrop : public Task
{
public:
	void init();
	int run();
private:
	int dropPos_;
	const int dropTol_ = 10;
	Leading_Edge_Latch dropStatusLEL_;
};

#endif // GRABBER_SET_DROP_H
