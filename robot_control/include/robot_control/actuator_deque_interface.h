#ifndef ACTUATOR_DEQUE_INTERFACE_H
#define ACTUATOR_DEQUE_INTERFACE_H
#include <deque>

class ActuatorDequeInterface
{
private:
	static std::deque driveDeque_;
	static std::deque grabberDeque_;
	static std::deque visionDeque_;
};

#endif // ACTUATOR_DEQUE_INTERFACE_H
