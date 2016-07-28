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
#include "vision_search.h"
#define TASK_POOL_SIZE 100
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
	static int driveDequeEnded;
	static int driveDequeEmpty;
	static int driveDequeEmptyPrev;
	static int grabberDequeEnded;
	static int grabberDequeEmpty;
	static int grabberDequeEmptyPrev;
	static int visionDequeEnded;
	static int visionDequeEmpty;
	static int visionDequeEmptyPrev;
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
int ActuatorDequeInterface::driveDequeEnded;
int ActuatorDequeInterface::driveDequeEmpty;
int ActuatorDequeInterface::driveDequeEmptyPrev = 1;
int ActuatorDequeInterface::grabberDequeEnded;
int ActuatorDequeInterface::grabberDequeEmpty;
int ActuatorDequeInterface::grabberDequeEmptyPrev = 1;
int ActuatorDequeInterface::visionDequeEnded;
int ActuatorDequeInterface::visionDequeEmpty;
int ActuatorDequeInterface::visionDequeEmptyPrev = 1;
#endif // ACTUATOR_DEQUE_INTERFACE_H
