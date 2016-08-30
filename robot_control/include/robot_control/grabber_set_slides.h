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
	int numFailedAttempts_;
	const int maxFailedAttempts_ = 2;
	const int limitedRetryPosition_ = GRABBER_CLOSED;
	const int failsafePosition_ = GRABBER_OPEN;
	bool limitedRetry_;
	enum SLIDES_STATE_T {_normal_, _recovering_} state_;
	int returnValue_;
};

#endif // GRABBER_SET_SLIDES_H
