#include <robot_control/reorient.h>

bool Reorient::runProc()
{
	switch(state)
	{
	case _init_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		procsToResume[procType] = false;
		clearSampleHistory();
		sendDriveRel(reorientDriveDistance, reorientPivotAngle, false, 0.0, false, false);
		sendDriveRel(0.0, -(90.0+reorientPivotAngle), false, 0.0, false, false);
		sendSearch(regionsOfInterestSrv.response.ROIList.at(currentROIIndex).whiteProb,
				   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).silverProb,
				   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).blueOrPurpleProb,
				   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).pinkProb,
				   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).redProb,
				   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).orangeProb,
				   regionsOfInterestSrv.response.ROIList.at(currentROIIndex).yellowProb);
		reorientCount++;
		state = _exec_;
		resetQueueEmptyCondition();
		break;
	case _exec_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		procsToResume[procType] = false;
		computeDriveSpeeds();
		if(searchEnded() || queueEmptyTimedOut) state = _finish_;
		else state = _exec_;
		serviceQueueEmptyCondition();
		break;
	case _interrupt_:
		avoidLockout = false;
		performReorient = false;
		sampleDataActedUpon = true;
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		procsToResume[procType] = false;
		state = _init_;
		break;
	case _finish_:
		avoidLockout = false;
		sampleDataActedUpon = true;
		performReorient = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		procsToResume[procType] = false;
		state = _init_;
		break;
	}
}
