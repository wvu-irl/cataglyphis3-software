#include <robot_control/avoid.h>

bool Avoid::runProc()
{
    //ROS_INFO("avoid state = %i",state);
	switch(state)
	{
	case _init_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        avoidCount++;
        if(avoidCount > maxAvoidCount)
        {
            //ROS_INFO("avoid count limit reached");
            sendDequeClearFront();
            dequeClearFront = false;
            avoidCount = 0;
            avoidLockout = true;
            procsBeingExecuted[procType] = false;
            procsToExecute[procType] = false;
            state = _init_;
            break;
        }
        computeDriveSpeeds();
		collisionInterruptThresh = (collisionMsg.distance_to_collision+collisionMinDistance)/2.0;
		intermediateWaypointsSrv.request.collision = collisionMsg.collision;
		intermediateWaypointsSrv.request.collisionDistance = collisionMsg.distance_to_collision;
		intermediateWaypointsSrv.request.current_x = robotStatus.xPos;
		intermediateWaypointsSrv.request.current_y = robotStatus.yPos;
		intermediateWaypointsSrv.request.current_heading = robotStatus.heading;
        intermediateWaypointsSrv.request.waypointArrayIn.resize(1);
		if(execInfoMsg.actionDeque[0]==_driveGlobal)
		{
            intermediateWaypointsSrv.request.waypointArrayIn.at(0).x = execInfoMsg.actionFloat1[0];
            intermediateWaypointsSrv.request.waypointArrayIn.at(0).y = execInfoMsg.actionFloat2[0];
		}
		else // _driveRelative // Else if? Don't want undefined behavior, but should not happen for anything other than driveGlobal and driveRelative
		{
            intermediateWaypointsSrv.request.waypointArrayIn.at(0).x = robotStatus.xPos + execInfoMsg.actionFloat1[0]*cos(execInfoMsg.actionFloat2[0]*DEG2RAD);
            intermediateWaypointsSrv.request.waypointArrayIn.at(0).y = robotStatus.yPos + execInfoMsg.actionFloat1[0]*sin(execInfoMsg.actionFloat2[0]*DEG2RAD);
		}
		if(intermediateWaypointsClient.call(intermediateWaypointsSrv)) ROS_DEBUG("intermediateWaypoints service call successful");
		else ROS_ERROR("intermediateWaypoints service call unsuccessful");
        if(intermediateWaypointsSrv.response.waypointArrayOut.size() > 0)
		{
            numWaypointsToTravel = intermediateWaypointsSrv.response.waypointArrayOut.size();
			clearAndResizeWTT();
            for(int i=0; i<numWaypointsToTravel; i++) waypointsToTravel.at(i) = intermediateWaypointsSrv.response.waypointArrayOut.at(numWaypointsToTravel-1-i);
            if(dequeClearFront)
            {
                sendDequeClearFront();
                dequeClearFront = false;
                ROS_INFO("avoid dequeClearFront true");
            }
            sendDriveGlobal(true, false, 0.0);
            //ROS_INFO("avoid sendDriveGlobal(front)");

		}
		state = _exec_;
		break;
	case _exec_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
		if(execLastProcType == procType && execLastSerialNum == serialNum) state = _finish_;
		else state = _exec_;
		break;
	case _interrupt_:
		avoidLockout = false;
        procsBeingExecuted[procType] = true;
		procsToInterrupt[procType] = false;
        dequeClearFront = true;
		state = _init_;
		break;
	case _finish_:
		avoidLockout = false;
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
        procsToResume[procType] = false;
		state = _init_;
		break;
	}
}
