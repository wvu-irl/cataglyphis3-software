#include <robot_control/grabber_set_drop.h>

void GrabberSetDrop::init()
{
	dropPos_ = params.int1;
}

int GrabberSetDrop::run()
{
    ROS_INFO("drop pos = %i",dropPos_);
	robotOutputs.dropPosCmd = dropPos_;
	robotOutputs.grabberStopCmd = 0;
	if(dropStatusLEL_.get_val()) return 1;
	else if(abs(robotStatus.grabberDropPos - dropPos_) <= dropTol_) return 1;
	else return 0;
}
