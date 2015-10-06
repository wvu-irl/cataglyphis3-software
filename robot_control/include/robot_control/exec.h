#ifndef EXEC_H
#define EXEC_H
#include <ros/ros.h>
#include <deque>
#include "action_type_enum.h"
#include "action.h"
#include "action_params.h"
#include "idle.h"
#include <messages/ExecAction.h>

#define ACTION_POOL_SIZE 10

class Exec
{
public:
	// Members
	ros::NodeHandle nh;
	ros::Publisher infoPub;
	ros::Subscriber actionSub;
	// Methods
	Exec(); // Constructor
	void run(); // Main run method for exec
private:
	// Members
	std::deque <Action*> actionDeque_;
    Idle pauseIdle;
	ACTION_TYPE_T nextActionType_ = _idle;
	bool newActionFlag_ = false;
	bool pushToFrontFlag_ = false;
	bool clearDequeFlag_ = false;
	bool pause_ = false;
	bool actionDequeEmpty_ = false;
	int currentActionDone_ = 1;
	size_t actionDequeSize_ = 0;
	unsigned int actionPoolIndex_[NUM_ACTIONS];
	Action* actionPool_[NUM_ACTIONS][ACTION_POOL_SIZE];
	ACTION_PARAMS_T params_;
	// Methods
	void actionCallback_(const messages::ExecAction::ConstPtr& msg);
};

#endif // EXEC_H
