#ifndef PROCEDURE_H
#define PROCEDURE_H
#include "mission_planning_process_share.h"

enum PROC_STATE_T {_init_, _exec_, _interrupt_, _finish_};

class Procedure : public MissionPlanningProcessShare
{
public:
    // Members
    PROC_TYPES_T procType;
    PROC_STATE_T state;
	unsigned int serialNum = 0;
    std::vector<robot_control::Waypoint>::iterator intermWaypointsIt;
    int initNumWaypointsToTravel;
	int totalIntermWaypoints;
    // Methods
    void reg(PROC_TYPES_T procTypeIn);
    bool run();
    virtual bool runProc() = 0;
    void clearAndResizeWTT();
    void callIntermediateWaypoints();
	void sendDriveGlobal(bool pushToFront);
	void sendDriveAndSearch(uint8_t typeMux);
    void sendDriveRel(float deltaDistance, float deltaHeading, bool endHeadingFlag, float endHeading, bool frontOfDeque);
	void sendSearch(uint8_t typeMux);
	void sendGrab();
	void sendDrop();
	void sendOpen();
};

void Procedure::reg(PROC_TYPES_T procTypeIn)
{
    this->procType = procTypeIn;
    this->state = _init_;
}

bool Procedure::run()
{
    //ROS_DEBUG("before if(procsToExecute.at(this->procType");
    //ROS_DEBUG("this->procType = %i",static_cast<int>(this->procType));
	if(procsToInterrupt[this->procType]) this->state = _interrupt_;
	if(procsToExecute[this->procType] || procsBeingExecuted[this->procType]) return this->runProc();
	//else if(procsBeingExecuted.at(this->procType) == true && procsToExecute.at(this->procType) == false) {this->state = _interrupt_; return this->runProc();}
    //ROS_DEBUG("after if - else if(procsToExecute.at(this->procType");
}

void Procedure::clearAndResizeWTT()
{
    waypointsToTravel.clear();
    waypointsToTravel.resize(numWaypointsToTravel);
}

void Procedure::callIntermediateWaypoints()
{
	intermediateWaypointsSrv.request.collision = collisionMsg.collision;
    initNumWaypointsToTravel = numWaypointsToTravel;
    totalIntermWaypoints = 0;
	intermediateWaypointsSrv.request.current_x = robotStatus.xPos;
	intermediateWaypointsSrv.request.current_y = robotStatus.yPos;
	intermediateWaypointsSrv.request.current_heading = robotStatus.heading;
    for(int i = 0; i < initNumWaypointsToTravel; i++)
    {
        intermWaypointsIt = waypointsToTravel.begin() + i + totalIntermWaypoints;
        if(i == 0)
        {
            intermediateWaypointsSrv.request.start.x = robotStatus.xPos;
            intermediateWaypointsSrv.request.start.y = robotStatus.yPos;
        }
        else
        {
            intermediateWaypointsSrv.request.start.x = (*(intermWaypointsIt - 1)).x;
            intermediateWaypointsSrv.request.start.y = (*(intermWaypointsIt - 1)).y;
        }
        intermediateWaypointsSrv.request.finish.x = (*intermWaypointsIt).x;
        intermediateWaypointsSrv.request.finish.y = (*intermWaypointsIt).y;
		if(intermediateWaypointsClient.call(intermediateWaypointsSrv)) ROS_DEBUG("intermediateWaypoints service call successful, size = %u", intermediateWaypointsSrv.response.waypointArray.size());
        else ROS_ERROR("intermediateWaypoints service call unsuccessful");
        if(intermediateWaypointsSrv.response.waypointArray.size() > 0)
        {
            waypointsToTravel.insert(intermWaypointsIt,intermediateWaypointsSrv.response.waypointArray.begin(),intermediateWaypointsSrv.response.waypointArray.end());
            totalIntermWaypoints += intermediateWaypointsSrv.response.waypointArray.size();
            numWaypointsToTravel += intermediateWaypointsSrv.response.waypointArray.size();
        }
    }
}

void Procedure::sendDriveGlobal(bool pushToFront)
{
    for(int i=0; i<numWaypointsToTravel; i++)
    {
		this->serialNum++;
        execActionSrv.request.nextActionType = _driveGlobal;
        execActionSrv.request.newActionFlag = 1;
		execActionSrv.request.pushToFrontFlag = pushToFront;
        execActionSrv.request.clearDequeFlag = false;
        execActionSrv.request.pause = false;
		execActionSrv.request.float1 = waypointsToTravel.at(i).x;
		execActionSrv.request.float2 = waypointsToTravel.at(i).y;
        execActionSrv.request.float3 = 1.5;
        execActionSrv.request.float4 = 45.0;
        execActionSrv.request.float5 = 0.0;
        execActionSrv.request.int1 = 0;
        execActionSrv.request.bool1 = false;
        execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
        execActionSrv.request.serialNum = this->serialNum;
        if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
        else ROS_ERROR("exec action service call unsuccessful");
    }
}

void Procedure::sendDriveAndSearch(uint8_t typeMux)
{
	for(int i=0; i<numWaypointsToTravel; i++)
	{
		// Drive Global
		this->serialNum++;
		execActionSrv.request.nextActionType = _driveGlobal;
		execActionSrv.request.newActionFlag = 1;
		execActionSrv.request.pushToFrontFlag = false;
		execActionSrv.request.clearDequeFlag = false;
		execActionSrv.request.pause = false;
		execActionSrv.request.float1 = waypointsToTravel.at(i).x;
		execActionSrv.request.float2 = waypointsToTravel.at(i).y;
		execActionSrv.request.float3 = 1.5;
		execActionSrv.request.float4 = 45.0;
		execActionSrv.request.float5 = 0.0;
		execActionSrv.request.int1 = 0;
		execActionSrv.request.bool1 = false;
		execActionSrv.request.bool2 = false;
		execActionSrv.request.bool3 = false;
		execActionSrv.request.bool4 = false;
		execActionSrv.request.bool5 = false;
		execActionSrv.request.bool6 = false;
		execActionSrv.request.bool7 = false;
		execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
		execActionSrv.request.serialNum = this->serialNum;
		if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
		else ROS_ERROR("exec action service call unsuccessful");
		// Search
		this->serialNum++;
		execActionSrv.request.nextActionType = _search;
		execActionSrv.request.newActionFlag = 1;
		execActionSrv.request.pushToFrontFlag = false;
		execActionSrv.request.clearDequeFlag = false;
		execActionSrv.request.pause = false;
		execActionSrv.request.float1 = 0.0;
		execActionSrv.request.float2 = 0.0;
		execActionSrv.request.float3 = 0.0;
		execActionSrv.request.float4 = 0.0;
		execActionSrv.request.float5 = 0.0;
		execActionSrv.request.int1 = 0;
		execActionSrv.request.bool1 = (typeMux & 1); // Purple
		execActionSrv.request.bool2 = (((typeMux & 2) >> 1) & 255); // Red
		execActionSrv.request.bool3 = (((typeMux & 4) >> 2) & 255); // Blue
		execActionSrv.request.bool4 = (((typeMux & 8) >> 3) & 255); // Silver
		execActionSrv.request.bool5 = (((typeMux & 16) >> 4) & 255); // Brass
		execActionSrv.request.bool6 = (((typeMux & 32) >> 5) & 255); // Confirm
		execActionSrv.request.bool7 = (((typeMux & 64) >> 6) & 255); // Save
		execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
		execActionSrv.request.serialNum = this->serialNum;
		if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
		else ROS_ERROR("exec action service call unsuccessful");
	}
}

void Procedure::sendDriveRel(float deltaDistance, float deltaHeading, bool endHeadingFlag, float endHeading, bool frontOfDeque)
{

    execActionSrv.request.nextActionType = _driveRelative;
    execActionSrv.request.newActionFlag = 1;
    execActionSrv.request.pushToFrontFlag = frontOfDeque;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.pause = false;
    execActionSrv.request.float1 = deltaDistance;
    execActionSrv.request.float2 = deltaHeading;
    execActionSrv.request.float3 = 1.5;
    execActionSrv.request.float4 = 45.0;
    execActionSrv.request.float5 = endHeading;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = endHeadingFlag;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendSearch(uint8_t typeMux)
{
	this->serialNum++;
	execActionSrv.request.nextActionType = _search;
	execActionSrv.request.newActionFlag = 1;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.float1 = 0.0;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = (typeMux & 1); // Purple
	execActionSrv.request.bool2 = (((typeMux & 2) >> 1) & 255); // Red
	execActionSrv.request.bool3 = (((typeMux & 4) >> 2) & 255); // Blue
	execActionSrv.request.bool4 = (((typeMux & 8) >> 3) & 255); // Silver
	execActionSrv.request.bool5 = (((typeMux & 16) >> 4) & 255); // Brass
	execActionSrv.request.bool6 = (((typeMux & 32) >> 5) & 255); // Confirm
	execActionSrv.request.bool7 = (((typeMux & 64) >> 6) & 255); // Save
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendGrab()
{
	execActionSrv.request.nextActionType = _grab;
	execActionSrv.request.newActionFlag = 1;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.float1 = 0.0;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
	this->serialNum++;
}

void Procedure::sendDrop()
{
	execActionSrv.request.nextActionType = _drop;
	execActionSrv.request.newActionFlag = 1;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.float1 = 0.0;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
	this->serialNum++;
}

void Procedure::sendOpen()
{
	execActionSrv.request.nextActionType = _open;
	execActionSrv.request.newActionFlag = 1;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.float1 = 0.0;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
	this->serialNum++;
}

#endif // PROCEDURE_H
