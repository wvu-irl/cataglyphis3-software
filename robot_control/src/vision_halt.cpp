#include <robot_control/vision_halt.h>

void VisionHalt::init()
{
	panAngle = robotStatus.panAngle;
	neckPos = robotStatus.neckPos;
}

int VisionHalt::run()
{
	//robotOutputs.panAngle = panAngle;
	//robotOutputs.neckPos = neckPos;
	robotOutputs.visionCmd = hold__;
	return 1;
}
