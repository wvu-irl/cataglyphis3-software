#include <robot_control/halt.h>

void Halt::init()
{
    clearDeques();
	visionHalt.init();
}

int Halt::run()
{
	driveHalt.run();
	grabberHalt.run();
	visionHalt.run();
}
