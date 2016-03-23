#include <robot_control/deposit_sample.h>

bool DepositSample::runProc()
{
	ROS_INFO("depositSampleState = %i", state);
	switch(state)
	{
	case _init_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		sendDrop();
		sendDriveRel(-3.0, 0.0, false, 0.0, false);
		state = _exec_;
		break;
	case _exec_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		if(execLastProcType == procType && execLastSerialNum == serialNum) state = _finish_;
		else state = _exec_;
		break;
	case _interrupt_:
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _init_;
		break;
	case _finish_:
		possessingSample = false;
		confirmedPossession = false;
		atHome = false;
		inDepositPosition = false;
		possibleSample = false;
		definiteSample = false;
		sampleDataActedUpon = false;
		sampleInCollectPosition = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		state = _init_;
		break;
	}
}
