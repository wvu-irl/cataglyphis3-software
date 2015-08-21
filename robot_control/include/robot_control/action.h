#ifndef ACTION_H
#define ACTION_H
#include "actuator_queue_interface.h"

//enum ACTION_TYPE_T {_stop, _travelWP, _drive, _pivot, _grab, _drop, _open, _search, _approach, _confirmCollect, _reorient, _deposit};

class Action : private ActuatorQueueInterface
{
public:
	virtual void init();
	virtual int run();
};

#endif // ACTION_H
