#include <robot_control/idle.h>

void Idle::init()
{
    clearDeques();
	visionHalt.init();
}

int Idle::run()
{
	driveHalt.run();
	grabberIdle.run();
	visionHalt.run();
}
