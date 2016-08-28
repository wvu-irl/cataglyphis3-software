#include <robot_control/halt.h>

void Halt::init()
{
    clearDeques();
	visionHalt.init();
    dropFailed_ = false;
    slidesFailed_ = false;
}

int Halt::run()
{
	driveHalt.run();
	grabberHalt.run();
	visionHalt.run();
}
