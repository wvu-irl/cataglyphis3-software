#ifndef PROCEDURE_H
#define PROCEDURE_H
#include "mission_planning_procedure_share.h"

class Procedure : public MissionPlanningProcedureShare
{
public:
    // Members
    PROC_TYPES_T procType;
    PROC_STATE_T state;
	unsigned int serialNum = 0;
    std::vector<robot_control::Waypoint>::iterator intermWaypointsIt;
    int initNumWaypointsToTravel;
	int totalIntermWaypoints;
	bool dequeClearFront = false;
    // Methods
    void reg(PROC_TYPES_T procTypeIn);
    bool run();
    virtual bool runProc() = 0;
    void clearAndResizeWTT();
    void callIntermediateWaypoints();
	void sendDriveGlobal(bool pushToFront, bool endHeadingFlag, float endHeading);
	void sendDriveAndSearch();
	void sendDriveAndWait(float waitTime, bool endHeadingFlag, float endHeading); // sec
	void sendDriveRel(float deltaDistance, float deltaHeading, bool endHeadingFlag, float endHeading, bool frontOfDeque, bool closedLoop);
	void sendSearch(float whiteProb, float silverProb, float blueOrPurpleProb, float pinkProb, float redProb, float orangeProb, float yellowProb);
	void sendGrab();
	void sendDrop();
	void sendOpen();
	void sendWait(float waitTime, bool pushToFront); // sec
	void sendDequeClearFront();
	void sendDequeClearAll();
	void sendPause();
	void sendUnPause();
	void findHighestConfSample();
	void computeSampleValuesWithExpectedDistance(bool useHistoryValue);
	void computeExpectedSampleLocation();
	void clearSampleHistory();
	void startSampleHistory(float& conf, bool* sampleTypes);
	bool sampleHistoryTypeMatch(bool* sampleTypes);
	void sampleHistoryNewData(float& conf, bool* sampleTypes);
	void sampleHistoryComputeModifiedConf();
	void computeDriveSpeeds();
	void serviceAvoidCounterDecrement();
	bool searchEnded();
	void resetQueueEmptyCondition();
	void serviceQueueEmptyCondition();
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
	if(procsToInterrupt[this->procType] && this->state==_exec_) {this->state = _interrupt_; procsToResume[this->procType] = true;}
	if((procsToResume[this->procType] && numProcsBeingOrToBeExec==0) || (procsToExecute[this->procType] || procsBeingExecuted[this->procType])) return this->runProc();
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
    initNumWaypointsToTravel = numWaypointsToTravel;
    totalIntermWaypoints = 0;
	intermediateWaypointsSrv.request.start_x = robotStatus.xPos;
	intermediateWaypointsSrv.request.start_y = robotStatus.yPos;
	intermediateWaypointsSrv.request.current_heading = robotStatus.heading;
	intermediateWaypointsSrv.request.waypointArrayIn.resize(numWaypointsToTravel);
	intermediateWaypointsSrv.request.waypointArrayIn = waypointsToTravel;
	if(intermediateWaypointsClient.call(intermediateWaypointsSrv)) ROS_DEBUG("intermediateWaypoints service call successful, size = %u", intermediateWaypointsSrv.response.waypointArrayOut.size());
	else ROS_ERROR("intermediateWaypoints service call unsuccessful");
	if(intermediateWaypointsSrv.response.waypointArrayOut.size() != numWaypointsToTravel)
	{
		waypointsToTravel.clear();
		waypointsToTravel = intermediateWaypointsSrv.response.waypointArrayOut;
		numWaypointsToTravel = waypointsToTravel.size();
		//for(int i=0; i<numWaypointsToTravel; i++) ROS_INFO("waypointsToTravel(%i) = (%f,%f)",i,waypointsToTravel.at(i).x,waypointsToTravel.at(i).y);
	}
	/*for(int i = 0; i < initNumWaypointsToTravel; i++)
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
	}*/
}

void Procedure::sendDriveGlobal(bool pushToFront, bool endHeadingFlag, float endHeading)
{
    for(int i=0; i<numWaypointsToTravel; i++)
    {
		this->serialNum++;
        execActionSrv.request.nextActionType = _driveGlobal;
        execActionSrv.request.newActionFlag = 1;
		execActionSrv.request.pushToFrontFlag = pushToFront;
        execActionSrv.request.clearDequeFlag = false;
		execActionSrv.request.clearFrontFlag = false;
        execActionSrv.request.pause = false;
		execActionSrv.request.pauseUnchanged = true;
		execActionSrv.request.float1 = waypointsToTravel.at(i).x;
		execActionSrv.request.float2 = waypointsToTravel.at(i).y;
		execActionSrv.request.float3 = endHeading;
		execActionSrv.request.float4 = 0.0;
        execActionSrv.request.float5 = 0.0;
		execActionSrv.request.float6 = 0.0;
		execActionSrv.request.float7 = 0.0;
		execActionSrv.request.int1 = waypointsToTravel.at(i).maxAvoids;
		if(i==(numWaypointsToTravel-1)) execActionSrv.request.bool1 = endHeadingFlag;
		else execActionSrv.request.bool1 = false;
		execActionSrv.request.bool2 = pushToFront;
		execActionSrv.request.bool3 = waypointsToTravel.at(i).unskippable;
		execActionSrv.request.bool4 = waypointsToTravel.at(i).roiWaypoint;
		execActionSrv.request.bool5 = false;
		execActionSrv.request.bool6 = false;
		execActionSrv.request.bool7 = false;
		execActionSrv.request.bool8 = false;
        execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
        execActionSrv.request.serialNum = this->serialNum;
        if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
        else ROS_ERROR("exec action service call unsuccessful");
    }
}

void Procedure::sendDriveAndSearch()
{
	for(int i=0; i<numWaypointsToTravel; i++)
	{
		// Drive Global
		this->serialNum++;
		execActionSrv.request.nextActionType = _driveGlobal;
		execActionSrv.request.newActionFlag = 1;
		execActionSrv.request.pushToFrontFlag = false;
		execActionSrv.request.clearDequeFlag = false;
		execActionSrv.request.clearFrontFlag = false;
		execActionSrv.request.pause = false;
		execActionSrv.request.pauseUnchanged = true;
		execActionSrv.request.float1 = waypointsToTravel.at(i).x;
		execActionSrv.request.float2 = waypointsToTravel.at(i).y;
		execActionSrv.request.float3 = 0.0;
		execActionSrv.request.float4 = 0.0;
		execActionSrv.request.float5 = 0.0;
		execActionSrv.request.float6 = 0.0;
		execActionSrv.request.float7 = 0.0;
		execActionSrv.request.int1 = waypointsToTravel.at(i).maxAvoids;
		execActionSrv.request.bool1 = false;
		execActionSrv.request.bool2 = false;
		execActionSrv.request.bool3 = waypointsToTravel.at(i).unskippable;
		execActionSrv.request.bool4 = waypointsToTravel.at(i).roiWaypoint;
		execActionSrv.request.bool5 = false;
		execActionSrv.request.bool6 = false;
		execActionSrv.request.bool7 = false;
		execActionSrv.request.bool8 = false;
		execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
		execActionSrv.request.serialNum = this->serialNum;
		if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
		else ROS_ERROR("exec action service call unsuccessful");
		// Search
		if(waypointsToTravel.at(i).searchable)
		{
			this->serialNum++;
			execActionSrv.request.nextActionType = _search;
			execActionSrv.request.newActionFlag = 1;
			execActionSrv.request.pushToFrontFlag = false;
			execActionSrv.request.clearDequeFlag = false;
			execActionSrv.request.clearFrontFlag = false;
			execActionSrv.request.pause = false;
			execActionSrv.request.pauseUnchanged = true;
			execActionSrv.request.float1 = waypointsToTravel.at(i).whiteProb; // white
			execActionSrv.request.float2 = waypointsToTravel.at(i).silverProb; // silver
			execActionSrv.request.float3 = waypointsToTravel.at(i).blueOrPurpleProb; // blueOrPurple
			execActionSrv.request.float4 = waypointsToTravel.at(i).pinkProb; // pink
			execActionSrv.request.float5 = waypointsToTravel.at(i).redProb; // red
			execActionSrv.request.float6 = waypointsToTravel.at(i).orangeProb; // orange
			execActionSrv.request.float7 = waypointsToTravel.at(i).yellowProb; // yellow
			execActionSrv.request.int1 = 0;
			execActionSrv.request.bool1 = false;
			execActionSrv.request.bool2 = false;
			execActionSrv.request.bool3 = false;
			execActionSrv.request.bool4 = false;
			execActionSrv.request.bool5 = false;
			execActionSrv.request.bool6 = false;
			execActionSrv.request.bool7 = false;
			execActionSrv.request.bool8 = false;
			execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
			execActionSrv.request.serialNum = this->serialNum;
			if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
			else ROS_ERROR("exec action service call unsuccessful");
		}
	}
}

void Procedure::sendDriveAndWait(float waitTime, bool endHeadingFlag, float endHeading)
{
	for(int i=0; i<numWaypointsToTravel; i++)
	{
		// Drive Global
		this->serialNum++;
		execActionSrv.request.nextActionType = _driveGlobal;
		execActionSrv.request.newActionFlag = 1;
		execActionSrv.request.pushToFrontFlag = false;
		execActionSrv.request.clearDequeFlag = false;
		execActionSrv.request.clearFrontFlag = false;
		execActionSrv.request.pause = false;
		execActionSrv.request.pauseUnchanged = true;
		execActionSrv.request.float1 = waypointsToTravel.at(i).x;
		execActionSrv.request.float2 = waypointsToTravel.at(i).y;
		execActionSrv.request.float3 = endHeading;
		execActionSrv.request.float4 = 0.0;
		execActionSrv.request.float5 = 0.0;
		execActionSrv.request.float6 = 0.0;
		execActionSrv.request.float7 = 0.0;
		execActionSrv.request.int1 = waypointsToTravel.at(i).maxAvoids;
		if(i==(numWaypointsToTravel-1)) execActionSrv.request.bool1 = endHeadingFlag;
		else execActionSrv.request.bool1 = false;
		execActionSrv.request.bool2 = false;
		execActionSrv.request.bool3 = waypointsToTravel.at(i).unskippable;
		execActionSrv.request.bool4 = waypointsToTravel.at(i).roiWaypoint;
		execActionSrv.request.bool5 = false;
		execActionSrv.request.bool6 = false;
		execActionSrv.request.bool7 = false;
		execActionSrv.request.bool8 = false;
		execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
		execActionSrv.request.serialNum = this->serialNum;
		if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
		else ROS_ERROR("exec action service call unsuccessful");
		// Wait
		this->serialNum++;
		execActionSrv.request.nextActionType = _wait;
		execActionSrv.request.newActionFlag = 1;
		execActionSrv.request.pushToFrontFlag = false;
		execActionSrv.request.clearDequeFlag = false;
		execActionSrv.request.clearFrontFlag = false;
		execActionSrv.request.pause = false;
		execActionSrv.request.pauseUnchanged = true;
		execActionSrv.request.float1 = waitTime;
		execActionSrv.request.float2 = 0.0;
		execActionSrv.request.float3 = 0.0;
		execActionSrv.request.float4 = 0.0;
		execActionSrv.request.float5 = 0.0;
		execActionSrv.request.float6 = 0.0;
		execActionSrv.request.float7 = 0.0;
		execActionSrv.request.int1 = 0;
		execActionSrv.request.bool1 = false;
		execActionSrv.request.bool2 = false;
		execActionSrv.request.bool3 = false;
		execActionSrv.request.bool4 = false;
		execActionSrv.request.bool5 = false;
		execActionSrv.request.bool6 = false;
		execActionSrv.request.bool7 = false;
		execActionSrv.request.bool8 = false;
		execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
		execActionSrv.request.serialNum = this->serialNum;
		if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
		else ROS_ERROR("exec action service call unsuccessful");
	}
}

void Procedure::sendDriveRel(float deltaDistance, float deltaHeading, bool endHeadingFlag, float endHeading, bool frontOfDeque, bool closedLoop)
{
	this->serialNum++;
    execActionSrv.request.nextActionType = _driveRelative;
    execActionSrv.request.newActionFlag = 1;
    execActionSrv.request.pushToFrontFlag = frontOfDeque;
    execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
    execActionSrv.request.float1 = deltaDistance;
    execActionSrv.request.float2 = deltaHeading;
	execActionSrv.request.float3 = endHeading;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
    execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = endHeadingFlag;
	execActionSrv.request.bool2 = frontOfDeque;
	execActionSrv.request.bool3 = closedLoop;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendSearch(float whiteProb, float silverProb, float blueOrPurpleProb, float pinkProb, float redProb, float orangeProb, float yellowProb)
{
	this->serialNum++;
	execActionSrv.request.nextActionType = _search;
	execActionSrv.request.newActionFlag = 1;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.clearFrontFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
	execActionSrv.request.float1 = whiteProb; // white
	execActionSrv.request.float2 = silverProb; // silver
	execActionSrv.request.float3 = blueOrPurpleProb; // blueOrPurple
	execActionSrv.request.float4 = pinkProb; // pink
	execActionSrv.request.float5 = redProb; // red
	execActionSrv.request.float6 = orangeProb; // orange
	execActionSrv.request.float7 = yellowProb; // yellow
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendGrab()
{
	this->serialNum++;
	execActionSrv.request.nextActionType = _grab;
	execActionSrv.request.newActionFlag = 1;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.clearFrontFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
	execActionSrv.request.float1 = 0.0;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendDrop()
{
	this->serialNum++;
	execActionSrv.request.nextActionType = _drop;
	execActionSrv.request.newActionFlag = 1;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.clearFrontFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
	execActionSrv.request.float1 = 0.0;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendOpen()
{
	this->serialNum++;
	execActionSrv.request.nextActionType = _open;
	execActionSrv.request.newActionFlag = 1;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.clearFrontFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
	execActionSrv.request.float1 = 0.0;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendWait(float waitTime, bool pushToFront)
{
	this->serialNum++;
	execActionSrv.request.nextActionType = _wait;
	execActionSrv.request.newActionFlag = 1;
	execActionSrv.request.pushToFrontFlag = pushToFront;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.clearFrontFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
	execActionSrv.request.float1 = waitTime;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendDequeClearFront()
{
	this->serialNum++;
	execActionSrv.request.nextActionType = 0;
	execActionSrv.request.newActionFlag = 0;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.clearFrontFlag = true;
	execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
	execActionSrv.request.float1 = 0.0;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
	ROS_INFO("send dequeClearFront");
	//voiceSay->call("queue clear front");
}

void Procedure::sendDequeClearAll()
{
	this->serialNum++;
	execActionSrv.request.nextActionType = 0;
	execActionSrv.request.newActionFlag = 0;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = true;
	execActionSrv.request.clearFrontFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
	execActionSrv.request.float1 = 0.0;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendPause()
{
    this->serialNum++;
    execActionSrv.request.nextActionType = 0;
    execActionSrv.request.newActionFlag = 0;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = true;
	execActionSrv.request.pauseUnchanged = false;
    execActionSrv.request.float1 = 0.0;
    execActionSrv.request.float2 = 0.0;
    execActionSrv.request.float3 = 0.0;
    execActionSrv.request.float4 = 0.0;
    execActionSrv.request.float5 = 0.0;
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = false;
    execActionSrv.request.bool2 = false;
    execActionSrv.request.bool3 = false;
    execActionSrv.request.bool4 = false;
    execActionSrv.request.bool5 = false;
    execActionSrv.request.bool6 = false;
    execActionSrv.request.bool7 = false;
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendUnPause()
{
    this->serialNum++;
    execActionSrv.request.nextActionType = 0;
    execActionSrv.request.newActionFlag = 0;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = false;
    execActionSrv.request.float1 = 0.0;
    execActionSrv.request.float2 = 0.0;
    execActionSrv.request.float3 = 0.0;
    execActionSrv.request.float4 = 0.0;
    execActionSrv.request.float5 = 0.0;
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = false;
    execActionSrv.request.bool2 = false;
    execActionSrv.request.bool3 = false;
    execActionSrv.request.bool4 = false;
    execActionSrv.request.bool5 = false;
    execActionSrv.request.bool6 = false;
    execActionSrv.request.bool7 = false;
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::findHighestConfSample()
{
	highestConfSample.confidence = 0.0;
	highestConfSample.distance = 0.0;
	highestConfSample.bearing = 0.0;
	for(int i=0; i<NUM_SAMPLE_TYPES; i++) highestConfSample.types[i] = false;
	for(int i=0; i<cvSamplesFoundMsg.sampleList.size(); i++)
	{
		if(cvSamplesFoundMsg.sampleList.at(i).confidence > highestConfSample.confidence)
		{
			highestConfSample.confidence = cvSamplesFoundMsg.sampleList.at(i).confidence;
			highestConfSample.distance = cvSamplesFoundMsg.sampleList.at(i).distance;
			highestConfSample.bearing = cvSamplesFoundMsg.sampleList.at(i).bearing;
			for(int j=0; j<NUM_SAMPLE_TYPES; j++)
			{
				switch(j) // !!! Change this switch case if NUM_SAMPLE_TYPES ever changed
				{
				case 0:
					highestConfSample.types[j] = cvSamplesFoundMsg.sampleList.at(i).white;
					break;
				case 1:
					highestConfSample.types[j] = cvSamplesFoundMsg.sampleList.at(i).silver;
					break;
				case 2:
					highestConfSample.types[j] = cvSamplesFoundMsg.sampleList.at(i).blueOrPurple;
					break;
				case 3:
					highestConfSample.types[j] = cvSamplesFoundMsg.sampleList.at(i).pink;
					break;
				case 4:
					highestConfSample.types[j] = cvSamplesFoundMsg.sampleList.at(i).red;
					break;
				case 5:
					highestConfSample.types[j] = cvSamplesFoundMsg.sampleList.at(i).orange;
					break;
				case 6:
					highestConfSample.types[j] = cvSamplesFoundMsg.sampleList.at(i).yellow;
					break;
				default:
					ROS_ERROR("findHighestConfSample tried to assign a sample type which does not exist");
					break;
				}
			}
		}
	}
}

void Procedure::computeSampleValuesWithExpectedDistance(bool useHistoryValue)
{
    float sampleConf;
    if(useHistoryValue) sampleConf = sampleHistoryModifiedConf;
    else sampleConf = highestConfSample.confidence;
	sampleDistanceAdjustedConf = sampleConfidenceGain*sampleConf -
							(sampleDistanceToExpectedGain*sqrt(pow(highestConfSample.distance,2.0)+pow(expectedSampleDistance,2.0)-
								2.0*highestConfSample.distance*expectedSampleDistance*
									cos(DEG2RAD*(highestConfSample.bearing-expectedSampleAngle))));
	ROS_INFO("^^^^^ sampleDistanceAdjustedConf = %f",sampleDistanceAdjustedConf);

	//ROS_INFO("after assigning sampleDistanceAdjustedConf");
}

void Procedure::computeExpectedSampleLocation()
{
	expectedSampleDistance = sqrt(pow(highestConfSample.distance,2) + pow(distanceToDrive,2) - 2.0*highestConfSample.distance*distanceToDrive*cos(DEG2RAD*(highestConfSample.bearing-angleToTurn)));
	if(distanceToDrive < highestConfSample.distance) expectedSampleAngle = highestConfSample.bearing - angleToTurn + RAD2DEG*asin(distanceToDrive/expectedSampleDistance*sin(DEG2RAD*(highestConfSample.bearing-angleToTurn)));
	else
	{
		if(highestConfSample.bearing >= 0.0) expectedSampleAngle = 180.0 - RAD2DEG*asin(highestConfSample.distance/expectedSampleDistance*sin(DEG2RAD*(highestConfSample.bearing-angleToTurn)));
		else expectedSampleAngle = -180.0 - RAD2DEG*asin(highestConfSample.distance/expectedSampleDistance*sin(DEG2RAD*(highestConfSample.bearing-angleToTurn)));
	}
}

void Procedure::clearSampleHistory()
{
	sampleHistoryActive = false;
	for(int i=0; i<NUM_SAMPLE_TYPES; i++) sampleHistoryTypes[i] = false;
	sampleHistoryGoodCount = 0;
	sampleHistoryBadCount = 0;
	sampleHistoryBestSampleConf = 0.0;
	sampleHistoryModifiedConf = 0.0;
	ROS_INFO("cleared sample history");
}

void Procedure::startSampleHistory(float& conf, bool* sampleTypes)
{
	if(!sampleHistoryActive)
	{
		if(conf >= possibleSampleConfThresh) possibleSample = true;
		else possibleSample = false;
		if(conf >= definiteSampleConfThresh) definiteSample = true;
		else definiteSample = false;
		if(possibleSample || definiteSample)
		{
			sampleHistoryActive = true;
			sampleHistoryBestSampleConf = conf;
			for(int i=0; i<NUM_SAMPLE_TYPES; i++) sampleHistoryTypes[i] = sampleTypes[i];
		}
	}
	else
	{
		ROS_ERROR("tried to start sample history when already started");
		possibleSample = false;
		definiteSample = false;
	}
}

bool Procedure::sampleHistoryTypeMatch(bool *sampleTypes)
{
	bool match = false;
	if(sampleHistoryActive)
	{
		for(int i=0; i<NUM_SAMPLE_TYPES; i++) if(sampleTypes[i] && sampleHistoryTypes[i]) match = true;
	}
	else ROS_ERROR("tried to check sample type match when sample history does not exist");
	return match;
}

void Procedure::sampleHistoryNewData(float& conf, bool* sampleTypes)
{
	if(sampleHistoryActive)
	{
		if(sampleHistoryTypeMatch(sampleTypes) && ((conf >= possibleSampleConfThresh) || (conf >= definiteSampleConfThresh)))
		{
			if(conf > sampleHistoryBestSampleConf) sampleHistoryBestSampleConf = conf;
			ROS_INFO("increment good count");
			sampleHistoryGoodCount++;
		}
		else {sampleHistoryBadCount--; ROS_INFO("increment bad count");}
		sampleHistoryComputeModifiedConf();
		if((sampleHistoryModifiedConf >= possibleSampleConfThresh) && (conf >= possibleSampleConfThresh)) possibleSample = true;
		else possibleSample = false;
		if((sampleHistoryModifiedConf >= definiteSampleConfThresh) && (conf >= definiteSampleConfThresh)) definiteSample = true;
		else definiteSample = false;
	}
	else ROS_ERROR("tried to incorporate new data into sample history when history does not exist");
}

void Procedure::sampleHistoryComputeModifiedConf()
{
	if(sampleHistoryActive)
	{
		sampleHistoryModifiedConf = sampleHistoryBestSampleConf + ((float)sampleHistoryGoodCount)*sampleHistoryGoodGain - ((float)sampleHistoryBadCount)*sampleHistoryBadGain;
		if(sampleHistoryModifiedConf > 1.0) {sampleHistoryModifiedConf = 1.0; sampleHistoryGoodCount--;}
		else if(sampleHistoryModifiedConf < 0.0) {sampleHistoryModifiedConf = 0.0; sampleHistoryBadCount--;}
	}
	else
	{
		ROS_ERROR("tried to compute sample history modified conf when sample history does not exist");
		sampleHistoryModifiedConf = 0.0;
		possibleSample = false;
		definiteSample = false;
	}
}

void Procedure::computeDriveSpeeds()
{
	if(collisionMsg.slowdown)
	{
		driveSpeedsMsg.vMax = slowVMax;
		driveSpeedsMsg.rMax = defaultRMax;
	}
	else
	{
		driveSpeedsMsg.vMax = defaultVMax;
		driveSpeedsMsg.rMax = defaultRMax;
	}
	if((driveSpeedsMsg.vMax != driveSpeedsMsgPrev.vMax) || (driveSpeedsMsg.rMax != driveSpeedsMsgPrev.rMax)) driveSpeedsPub.publish(driveSpeedsMsg);
	driveSpeedsMsgPrev.vMax = driveSpeedsMsg.vMax;
	driveSpeedsMsgPrev.rMax = driveSpeedsMsg.rMax;
}

void Procedure::serviceAvoidCounterDecrement()
{
	if(hypot(robotStatus.xPos - prevAvoidCountDecXPos, robotStatus.yPos - prevAvoidCountDecYPos) > metersPerAvoidCountDecrement)
	{
		if(avoidCount > 0) avoidCount--;
		prevAvoidCountDecXPos = robotStatus.xPos;
		prevAvoidCountDecYPos = robotStatus.yPos;
	}
}

bool Procedure::searchEnded()
{
	if((cvSamplesFoundMsg.procType==this->procType && cvSamplesFoundMsg.serialNum==this->serialNum) || searchTimedOut)
	{
		ROS_INFO("searchEnded return true");
		timers[_searchTimer_]->stop();
		searchTimedOut = false;
		return true;
	}
	else return false;
}

void Procedure::resetQueueEmptyCondition()
{
	timers[_queueEmptyTimer_]->stop();
	queueEmptyTimedOut = false;
	ROS_INFO("reset queue empty timer");
}

void Procedure::serviceQueueEmptyCondition()
{
	if(execInfoMsg.actionDequeSize==0 && !timers[_queueEmptyTimer_]->running && !queueEmptyTimedOut) {timers[_queueEmptyTimer_]->start(); ROS_WARN("start queue empty timer");}
	else if(execInfoMsg.actionDequeSize>0 && timers[_queueEmptyTimer_]->running) {timers[_queueEmptyTimer_]->stop(); queueEmptyTimedOut = false; ROS_INFO("stop queue empty timer");}
}

#endif // PROCEDURE_H
