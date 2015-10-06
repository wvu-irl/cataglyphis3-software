#ifndef ACTUATOR_DEQUE_INTERFACE_H
#define ACTUATOR_DEQUE_INTERFACE_H
#include <deque>
#include "task.h"

class ActuatorDequeInterface
{
private:
    static std::deque <Task*> driveDeque_;
    static std::deque <Task*> grabberDeque_;
    static std::deque <Task*> visionDeque_;
};

#endif // ACTUATOR_DEQUE_INTERFACE_H
