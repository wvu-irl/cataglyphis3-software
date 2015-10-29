#include <robot_control/idle.h>

void Idle::init()
{
	driveDeque.clear();
	grabberDeque.clear();
	visionDeque.clear();
	visionHalt.init();
}

int Idle::run()
{
	driveHalt.run();
	grabberIdle.run();
	visionHalt.run();
}
