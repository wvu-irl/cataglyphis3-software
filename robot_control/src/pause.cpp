#include <robot_control/pause.h>

bool Pause::runProc()
{
    return true;
}

void Pause::sendPause()
{
    this->serialNum++;
    execActionSrv.request.nextActionType = 0;
    execActionSrv.request.newActionFlag = 0;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = true;
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
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Pause::sendUnPause()
{
    this->serialNum++;
    execActionSrv.request.nextActionType = 0;
    execActionSrv.request.newActionFlag = 0;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
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
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}
