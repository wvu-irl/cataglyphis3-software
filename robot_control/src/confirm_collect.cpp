#include <robot_control/confirm_collect.h>

bool ConfirmCollect::runProc()
{
	ROS_INFO("confirmCollectState = %i", state);
	switch(state)
	{
	case _init_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		sampleTypeMux = 0;
		//sampleTypeMux = (1 << (static_cast<uint8_t>(bestSample.type) & 255)) & 255;
		//ROS_INFO("confirmCollect sampleTypeMux = %u",sampleTypeMux);
		//sendSearch(sampleTypeMux);
		sendSearch(124); // 124 = b1111100 -> purple = 1; red = 1; blue = 1; silver = 1; brass = 1; confirm = 0; save = 0;
		state = _exec_;
		break;
	case _exec_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		if(cvSamplesFoundMsg.procType==this->procType && cvSamplesFoundMsg.serialNum==this->serialNum &&
				execLastProcType == procType && execLastSerialNum == serialNum)
		{
			if(!definiteSample) // Make smarter by looking for lack of sample close to expected location, similar to how it is done in approach
			{
				confirmedPossession = true;
				state = _finish_;
			}
			else
			{
				confirmedPossession = false;
				possessingSample = false;
				sendOpen();
				state = _finish_;
			}
		}
		else state = _exec_;
		break;
	case _interrupt_:
		sampleDataActedUpon = true;
		sampleInCollectPosition = false;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		state = _init_;
		break;
	case _finish_:
		sampleDataActedUpon = true;
		sampleInCollectPosition = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		state = _init_;
		break;
	}
}
