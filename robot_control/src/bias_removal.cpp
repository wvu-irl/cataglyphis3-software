#include <robot_control/bias_removal.h>

bool BiasRemoval::runProc()
{
    //ROS_INFO("biasRemoval state = %i", state);
	switch(state)
	{
	case _init_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        if(!tiltTooExtremeForBiasRemoval)
        {
            sendPause();
            biasRemovalTimedOut = false;
            timers[_biasRemovalActionTimeoutTimer_]->start();
            navControlSrv.request.runBiasRemoval = true;
            if(navControlClient.call(navControlSrv)) ROS_DEBUG("navFilterControlService call successful");
            else ROS_ERROR("navFilterControlService call unsuccessful");
            state = _exec_;
        }
        else
        {
        	biasRemovalTimedOut = false;
            voiceSay->call("Tilt too extreme for bias removal. Moving on.");
            state = _finish_;
        }
        resetQueueEmptyCondition();
		break;
	case _exec_:
		avoidLockout = true;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        if(navStatus!=0 || biasRemovalTimedOut || queueEmptyTimedOut) state = _finish_;
        else state = _exec_;
        serviceQueueEmptyCondition();
		break;
	case _interrupt_:
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		sendUnPause();
		timers[_biasRemovalActionTimeoutTimer_]->stop();
		state = _init_;
		break;
	case _finish_:
		avoidLockout = false;
		performBiasRemoval = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		sendUnPause();
		timers[_biasRemovalActionTimeoutTimer_]->stop();
		timers[_biasRemovalTimer_]->stop();
		timers[_biasRemovalTimer_]->start();
		state = _init_;
		break;
	}
}
