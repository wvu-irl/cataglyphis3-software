#include <robot_control/grab.h>

void Grab::init()
{
	driveDeque.clear();
	grabberDeque.clear();
	visionDeque.clear();
	pushTask(_grabberSetDrop_);
	grabberDeque.front()->params.int1 = GRABBER_DROPPED;
	pushTask(_grabberSetSlides_);
	grabberDeque.front()->params.int1 = GRABBER_CLOSED;
	pushTask(_grabberSetDrop_);
	grabberDeque.front()->params.int1 = GRABBER_RAISED;
}

int Grab::run()
{
	return runDeques();
}
