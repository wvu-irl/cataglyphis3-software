#include <robot_control/open.h>

void Open::init()
{
    clearDeques();
	pushTask(_grabberSetSlides_);
	grabberDeque.back()->params.int1 = GRABBER_OPEN;
}

int Open::run()
{
	runDeques();
	return 1; // Other actions do not have to wait on grabber opening to continue. End immediately to save mission time.
}
