#include <robot_control/grabber_idle.h>

void GrabberIdle::init()
{
	robotOutputs.grabberStopCmd = 0;
}

int GrabberIdle::run()
{
	robotOutputs.grabberStopCmd = 0;
	return 1;
}
