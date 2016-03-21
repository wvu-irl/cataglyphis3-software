#include <robot_control/drop.h>

void Drop::init()
{
    clearDeques();
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
