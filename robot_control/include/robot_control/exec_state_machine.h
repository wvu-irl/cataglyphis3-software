#ifndef EXEC_STATE_MACHINE_H
#define EXEC_STATE_MACHINE_H
#include <ros/ros.h>
#include <queue>
#include "action_type_enum.h"
#include "action.h"

#define ACTION_POOL_SIZE 10

class ExecStateMachine
{
public:
	// Members
	ros::NodeHandle nh;
	ros::Publisher infoPub;
	ros::Subscriber actionSub;
	// Methods
	ExecStateMachine(); // Constructor
	void run(); // Main run method for exec state machine
private:
	// Members
	std::queue <Action*> actionQueue_;
	Action* currentAction_;
	//Idle pauseIdle;
	ACTION_TYPE_T nextActionType_ = _idle;
	bool newActionFlag_ = false;
	bool clearQueueFlag_ = false;
	bool pause_ = false;
	bool actionQueueEmpty_ = false;
	bool currentActionDone_ = true;
	size_t actionQueueSize_ = 0;
	unsigned int actionPoolIndex_[NUM_ACTIONS];
	Action* actionPool_[NUM_ACTIONS][ACTION_POOL_SIZE];
	// Methods
	void actionCallback_();
};

#endif // EXEC_STATE_MACHINE_H
