#include <robot_control/drop.h>

void Drop::init()
{
	driveDeque.clear();
	grabberDeque.clear();
	visionDeque.clear();
	pushTask(_grabberSetDrop_);
	grabberDeque.front()->params.int1 = GRABBER_DROPPED;
	pushTask(_grabberSetSlides_);
	grabberDeque.front()->params.int1 = GRABBER_OPEN;
	pushTask(_grabberSetDrop_);
	grabberDeque.front()->params.int1 = GRABBER_RAISED;
}

int Drop::run()
{
	return runDeques();
}
