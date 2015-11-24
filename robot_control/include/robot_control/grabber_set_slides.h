#ifndef GRABBER_SET_SLIDES_H
#define GRABBER_SET_SLIDES_H
#include "task.h"

class GrabberSetSlides : public Task
{
public:
	void init();
	int run();
private:
	int slidePos_;
	const int slideTol_ = 10;
	Leading_Edge_Latch slideStatusLEL_;
};

#endif // GRABBER_SET_SLIDES_H
