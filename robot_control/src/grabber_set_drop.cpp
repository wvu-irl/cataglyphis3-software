#include <robot_control/grabber_set_drop.h>

void GrabberSetDrop::init()
{
	dropPos_ = params.int1;
}

int GrabberSetDrop::run()
{
	robotOutputs.dropPosCmd = dropPos_;
	robotOutputs.grabberStopCmd = 0;
	dropStatusLEL_.LE_Latch(robotStatus.grabberDropStatus);
	if(dropStatusLEL_.get_val()) return 1;
	else return 0;
}
