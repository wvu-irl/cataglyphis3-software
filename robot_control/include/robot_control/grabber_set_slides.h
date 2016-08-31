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
	int slideRecoveryPos_;
	int numFailedAttempts_;
	int slidesFinishedCount_;
	int slidesStuckOpenWaitCount_;
	const int maxSlidesFinishedCount_ = 300; // 15 sec = 300 counts at 20 hz
	const int maxSlidesStuckOpenWaitCount_ = 200; //  10 sec = 200 counts at 2- hz
	const int maxFailedAttempts_ = 2;
	const int limitedRetryPosition_ = GRABBER_CLOSED;
	const int failsafePosition_ = GRABBER_OPEN;
	bool limitedRetry_;
	enum SLIDES_STATE_T {_normal_, _recovering_, _stuckOpenWait_} state_;
	int returnValue_;
};

#endif // GRABBER_SET_SLIDES_H
