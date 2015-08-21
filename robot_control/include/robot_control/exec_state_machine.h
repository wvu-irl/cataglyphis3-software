#ifndef EXEC_STATE_MACHINE_H
#define EXEC_STATE_MACHINE_H
#include <ros/ros.h>
#include <queue>

class ExecStateMachine
{
public:
	// Members
	ros::NodeHandle nh;
	ros::Publisher infoPub;
	ros::Subscriber actionSub;
	// Methods
	ExecStateMachine(); // Constructor
private:
	// Members
	std::queue actionQueue_;
	// Methods
	void actionCallback_();
};

#endif // EXEC_STATE_MACHINE_H
