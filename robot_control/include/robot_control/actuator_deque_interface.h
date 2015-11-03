#ifndef ACTUATOR_DEQUE_INTERFACE_H
#define ACTUATOR_DEQUE_INTERFACE_H
#include <deque>
#include "task.h"
#include "task_type_enum.h"
#include "drive_halt.h"
#include "drive_straight.h"
#include "drive_pivot.h"
#include "drive_straight_cl.h"
#include "grabber_halt.h"
#include "grabber_set_drop.h"
#include "grabber_set_slides.h"
#include "grabber_idle.h"
#include "vision_halt.h"
#define TASK_POOL_SIZE 50
#define PI 3.14159265359

class ActuatorDequeInterface
{
public:
	static std::deque <Task*> driveDeque;
	static std::deque <Task*> grabberDeque;
	static std::deque <Task*> visionDeque;
	static unsigned int taskPoolIndex[NUM_TASKS];
	static Task* taskPool[NUM_TASKS][TASK_POOL_SIZE];
	static DriveHalt driveHalt;
	static GrabberHalt grabberHalt;
	static GrabberIdle grabberIdle;
	static VisionHalt visionHalt;
};

std::deque <Task*> ActuatorDequeInterface::driveDeque;
std::deque <Task*> ActuatorDequeInterface::grabberDeque;
std::deque <Task*> ActuatorDequeInterface::visionDeque;
unsigned int ActuatorDequeInterface::taskPoolIndex[NUM_TASKS] = {0};
Task* ActuatorDequeInterface::taskPool[NUM_TASKS][TASK_POOL_SIZE] = {0};
DriveHalt ActuatorDequeInterface::driveHalt;
GrabberHalt ActuatorDequeInterface::grabberHalt;
GrabberIdle ActuatorDequeInterface::grabberIdle;
VisionHalt ActuatorDequeInterface::visionHalt;

#endif // ACTUATOR_DEQUE_INTERFACE_H
