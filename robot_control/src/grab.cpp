#include <robot_control/grab.h>

void Grab::init()
{
    clearDeques();
    dropFailed_ = false;
    slidesFailed_ = false;
	pushTask(_grabberSetDrop_);
	grabberDeque.back()->params.int1 = GRABBER_DROPPED;
	pushTask(_grabberSetSlides_);
	grabberDeque.back()->params.int1 = GRABBER_CLOSED;
	pushTask(_grabberSetDrop_);
	grabberDeque.back()->params.int1 = GRABBER_RAISED;
}

int Grab::run()
{
	return runDeques();
}
