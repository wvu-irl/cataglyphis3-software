#include <robot_control/grabber_halt.h>

void GrabberHalt::init()
{
    robotOutputs.grabberStopCmd = 1;
}

int GrabberHalt::run()
{
	robotOutputs.grabberStopCmd = 1;
	return 1;
}
