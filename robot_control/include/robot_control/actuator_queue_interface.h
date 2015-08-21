#ifndef ACTUATOR_QUEUE_INTERFACE_H
#define ACTUATOR_QUEUE_INTERFACE_H
#include <queue>

class ActuatorQueueInterface
{
private:
	static std::queue driveQueue_;
	static std::queue grabberQueue_;
	static std::queue visionQueue_;
};

#endif // ACTUATOR_QUEUE_INTERFACE_H
