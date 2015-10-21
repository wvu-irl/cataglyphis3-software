#ifndef ACTION_H
#define ACTION_H
#include "actuator_deque_interface.h"
#include "action_params.h"
#include "robot_control_interface.h"
#include <math.h>

class Action : public ActuatorDequeInterface, public RobotControlInterface
{
public:
	virtual void init() = 0;
	virtual int run() = 0;
	ACTION_PARAMS_T params;
	void pushTask(TASK_TYPE_T taskType);
	int runDeques();
	int driveDequeEnded;
	int driveDequeEmpty;
	int driveDequeEmptyPrev = 1;
	int grabberDequeEnded;
	int grabberDequeEmpty;
	int grabberDequeEmptyPrev = 1;
	int visionDequeEnded;
	int visionDequeEmpty;
	int visionDequeEmptyPrev = 1;
};

#endif // ACTION_H
