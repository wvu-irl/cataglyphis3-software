#ifndef ACTION_H
#define ACTION_H
#include "actuator_queue_interface.h"

class Action : private ActuatorQueueInterface
{
public:
	virtual void init() = 0;
	virtual int run() = 0;
};

#endif // ACTION_H
