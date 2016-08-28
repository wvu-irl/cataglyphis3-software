#include <robot_control/drop.h>

void Drop::init()
{
    clearDeques();
    dropFailed_ = false;
    slidesFailed_ = false;
	pushTask(_grabberSetDrop_);
	grabberDeque.back()->params.int1 = GRABBER_DROPPED;
	pushTask(_grabberSetSlides_);
	grabberDeque.back()->params.int1 = GRABBER_OPEN;
	pushTask(_grabberSetDrop_);
	grabberDeque.back()->params.int1 = GRABBER_RAISED;
}

int Drop::run()
{
	return runDeques();
}
