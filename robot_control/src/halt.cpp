#include <robot_control/halt.h>

void Halt::init()
{
	driveDeque.clear();
	grabberDeque.clear();
	visionDeque.clear();
	visionHalt.init();
}

int Halt::run()
{
	driveHalt.run();
	grabberHalt.run();
	visionHalt.run();
}
