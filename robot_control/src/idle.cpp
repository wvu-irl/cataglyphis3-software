#include <robot_control/idle.h>

void Idle::init()
{
	driveDeque.clear();
	grabberDeque.clear();
	visionDeque.clear();
	visionEmptyHalt.init();
}

int Idle::run()
{
	driveEmptyHalt.run();
	grabberEmptyHalt.run();
	visionEmptyHalt.run();
}
