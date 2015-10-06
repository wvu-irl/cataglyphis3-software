#ifndef ACTUATOR_DEQUE_INTERFACE_H
#define ACTUATOR_DEQUE_INTERFACE_H
#include <deque>
#include "task.h"
#include "task_type_enum.h"
#include "drive_halt.h"
#include "grabber_halt.h"
#include "vision_halt.h"
#define TASK_POOL_SIZE 50

class ActuatorDequeInterface
{
public:
	static std::deque <Task*> driveDeque;
	static std::deque <Task*> grabberDeque;
	static std::deque <Task*> visionDeque;
	static unsigned int taskPoolIndex[NUM_TASKS];
	static Task* taskPool[NUM_TASKS][TASK_POOL_SIZE];
	static DriveHalt driveEmptyHalt;
	static GrabberHalt grabberEmptyHalt;
	static VisionHalt visionEmptyHalt;
};

#endif // ACTUATOR_DEQUE_INTERFACE_H
