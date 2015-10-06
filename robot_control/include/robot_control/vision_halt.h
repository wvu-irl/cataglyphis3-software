#ifndef VISION_HALT_H
#define VISION_HALT_H
#include "task.h"

class VisionHalt : public Task
{
public:
	void init();
	int run();
	float panAngle;
	float neckPos;
};

#endif // VISION_HALT_H
