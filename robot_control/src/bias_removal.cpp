#include <robot_control/bias_removal.h>

BiasRemoval::BiasRemoval()
{
	timers[_biasRemovalActionTimeoutTimer_] = new CataglyphisTimer<BiasRemoval>(&BiasRemoval::callback, this);
}

bool BiasRemoval::runProc()
{
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		sendDequeClearAll();
		biasRemovalTimedOut = false;
		timers[_biasRemovalActionTimeoutTimer_]->setPeriod(biasRemovalTimeoutPeriod);
		timers[_biasRemovalActionTimeoutTimer_]->start();
		navControlSrv.request.runBiasRemoval = true;
		if(navControlClient.call(navControlSrv)) ROS_DEBUG("navFilterControlService call successful");
		else ROS_ERROR("navFilterControlService call unsuccessful");
		state = _exec_;
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		if(navStatus!=0 || biasRemovalTimedOut) state = _finish_;
		else state = _exec_;
		break;
	case _interrupt_:
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		timers[_biasRemovalActionTimeoutTimer_]->stop();
		state = _init_;
		break;
	case _finish_:
		avoidLockout = false;
		performBiasRemoval = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		timers[_biasRemovalActionTimeoutTimer_]->stop();
		timers[_biasRemovalTimer_]->stop();
		timers[_biasRemovalTimer_]->start();
		state = _init_;
		break;
	}
}

void BiasRemoval::callback(const ros::TimerEvent &event)
{
	biasRemovalTimedOut = true;
}
