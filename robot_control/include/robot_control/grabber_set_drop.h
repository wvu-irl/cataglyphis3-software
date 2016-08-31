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
	int dropRecoveryPos_;
	int numFailedAttempts_;
	const int maxFailedAttempts_ = 2;
	int dropFinishedCount_;
	const int maxDropFinishedCount_ = 300; // counts at 20 hz
	const int limitedRetryPosition_ = GRABBER_DROPPED;
	const int failsafePosition_ = GRABBER_RAISED;
	bool limitedRetry_;
	enum DROP_STATE_T {_normal_, _recovering_} state_;
	int returnValue_;
};

#endif // GRABBER_SET_DROP_H
