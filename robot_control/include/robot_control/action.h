#ifndef ACTION_H
#define ACTION_H
#include "actuator_deque_interface.h"
#include "action_params.h"

class Action : private ActuatorDequeInterface
{
public:
	virtual void init() = 0;
	virtual int run() = 0;
	ACTION_PARAMS_T params;
};

#endif // ACTION_H
