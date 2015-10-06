#ifndef DRIVE_HALT_H
#define DRIVE_HALT_H
#include "task.h"

class DriveHalt : public Task
{
public:
	void init();
	int run();
};

#endif // DRIVE_HALT_H
