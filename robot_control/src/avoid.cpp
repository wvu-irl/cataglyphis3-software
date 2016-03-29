#include <robot_control/avoid.h>

bool Avoid::runProc()
{
	ROS_INFO("avoid state = %i",state);
	switch(state)
	{
	case _init_:
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		collisionInterruptThresh = (collisionMsg.distance_to_collision+collisionMinDistance)/2.0;
		intermediateWaypointsSrv.request.collision = collisionMsg.collision;
		intermediateWaypointsSrv.request.collisionDistance = collisionMsg.distance_to_collision;
		intermediateWaypointsSrv.request.start.x = robotStatus.xPos;
		intermediateWaypointsSrv.request.start.y = robotStatus.yPos;
		intermediateWaypointsSrv.request.current_x = robotStatus.xPos;
		intermediateWaypointsSrv.request.current_y = robotStatus.yPos;
		intermediateWaypointsSrv.request.current_heading = robotStatus.heading;
		if(execInfoMsg.actionDeque[0]==_driveGlobal)
		{
			intermediateWaypointsSrv.request.finish.x = execInfoMsg.actionFloat1[0];
			intermediateWaypointsSrv.request.finish.y = execInfoMsg.actionFloat2[0];
		}
		else // _driveRelative // Else if? Don't want undefined behavior, but should not happen for anything other than driveGlobal and driveRelative
		{
			intermediateWaypointsSrv.request.finish.x = robotStatus.xPos + execInfoMsg.actionFloat1[0]*cos(execInfoMsg.actionFloat2[0]*DEG2RAD);
			intermediateWaypointsSrv.request.finish.y = robotStatus.yPos + execInfoMsg.actionFloat1[0]*sin(execInfoMsg.actionFloat2[0]*DEG2RAD);
		}
		if(intermediateWaypointsClient.call(intermediateWaypointsSrv)) ROS_DEBUG("intermediateWaypoints service call successful");
		else ROS_ERROR("intermediateWaypoints service call unsuccessful");
		if(intermediateWaypointsSrv.response.waypointArray.size() > 0)
		{
			numWaypointsToTravel = intermediateWaypointsSrv.response.waypointArray.size();
			clearAndResizeWTT();
			for(int i=0; i<numWaypointsToTravel; i++) waypointsToTravel.at(i) = intermediateWaypointsSrv.response.waypointArray.at(numWaypointsToTravel-1-i);
			sendDriveGlobal(true, interrupted);
			if(interrupted) interrupted = false;
		}
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
		interrupted = true;
		state = _init_;
		break;
	case _finish_:
		procsBeingExecuted[procType] = false;
		procsToExecute[procType] = false;
		state = _init_;
		break;
	}
}
