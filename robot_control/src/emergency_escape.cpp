#include <robot_control/emergency_escape.h>

bool EmergencyEscape::runProc()
{
	ROS_INFO("emergency escape state = %i",state);
	switch(state)
	{
	case _init_:
        if(escapeCondition)
        {
            escapeLockout = true;
            procsBeingExecuted[procType] = true;
            procsToExecute[procType] = false;
            procsToResume[procType] = false;
            if(interrupted)
            {
                sendDequeClearFront();
                interrupted = false;
            }
            sendDriveRel(offsetDistance, offsetAngle, false, 0.0, true);
            offsetDriveSerialNum = serialNum;
            sendDriveRel(backupDistance, 0.0, false, 0.0, true);
            backupSerialNum = serialNum;
            state = _exec_;
        }
        else
        {
            procsBeingExecuted[procType] = false;
            procsToExecute[procType] = false;
            procsToResume[procType] = false;
            interrupted = false;
            state = _init_;
        }
		break;
	case _exec_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        //ROS_INFO("execLastSerialNum = %u",execLastSerialNum);
        //ROS_INFO("backupSerialNum = %u", backupSerialNum);
        //ROS_INFO("offsetDriveSerialNum = %u", offsetDriveSerialNum);
		if(execLastProcType == procType && execLastSerialNum == backupSerialNum) escapeLockout = false;
		if(execLastProcType == procType && execLastSerialNum == offsetDriveSerialNum) state = _finish_;
		else state = _exec_;
		break;
	case _interrupt_:
		procsBeingExecuted[procType] = false;
		procsToInterrupt[procType] = false;
		interrupted = true;
		state = _init_;
		break;
	case _finish_:
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		state = _init_;
		break;
	}
}
