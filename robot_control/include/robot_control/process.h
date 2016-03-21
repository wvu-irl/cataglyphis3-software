#ifndef PROCESS_H
#define PROCESS_H
#include "mission_planning_process_share.h"

enum PROC_STATE_T {_init_, _exec_, _interrupt_, _finish_};

class Process : public MissionPlanningProcessShare
{
public:
    // Members
    PROC_TYPES_T procType;
    PROC_STATE_T state;
    unsigned int serialNum;
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
	void sendDriveAndSearch(bool purple, bool red, bool blue, bool silver, bool brass, bool confirm, bool save);
    void sendDriveRel(float deltaDistance, float deltaHeading, bool endHeadingFlag, float endHeading, bool frontOfDeque);
};

void Process::reg(PROC_TYPES_T procTypeIn)
{
    this->procType = procTypeIn;
    this->state = _init_;
}

bool Process::run()
{
    //ROS_DEBUG("before if(procsToExecute.at(this->procType");
    //ROS_DEBUG("this->procType = %i",static_cast<int>(this->procType));
	if(procsToInterrupt[this->procType]) this->state = _interrupt_;
	if(procsToExecute[this->procType] || procsBeingExecuted[this->procType]) return this->runProc();
	//else if(procsBeingExecuted.at(this->procType) == true && procsToExecute.at(this->procType) == false) {this->state = _interrupt_; return this->runProc();}
    //ROS_DEBUG("after if - else if(procsToExecute.at(this->procType");
}

void Process::clearAndResizeWTT()
{
    waypointsToTravel.clear();
    waypointsToTravel.resize(numWaypointsToTravel);
}

void Process::callIntermediateWaypoints()
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

void Process::sendDriveGlobal(bool pushToFront)
{
    for(int i=0; i<numWaypointsToTravel; i++)
    {
        this->serialNum = i;
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

void Process::sendDriveAndSearch(bool purple, bool red, bool blue, bool silver, bool brass, bool confirm, bool save)
{
	this->serialNum = 0;
	for(int i=0; i<numWaypointsToTravel; i++)
	{
		// Drive Global
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
		this->serialNum++;
		// Search
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
		execActionSrv.request.bool1 = purple;
		execActionSrv.request.bool2 = red;
		execActionSrv.request.bool3 = blue;
		execActionSrv.request.bool4 = silver;
		execActionSrv.request.bool5 = brass;
		execActionSrv.request.bool6 = confirm;
		execActionSrv.request.bool7 = save;
		execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
		execActionSrv.request.serialNum = this->serialNum;
		if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
		else ROS_ERROR("exec action service call unsuccessful");
		this->serialNum++;
	}
	this->serialNum--;
}

void Process::sendDriveRel(float deltaDistance, float deltaHeading, bool endHeadingFlag, float endHeading, bool frontOfDeque)
{
	this->serialNum = 0;
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

#endif // PROCESS_H
