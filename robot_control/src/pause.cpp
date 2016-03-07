#include <robot_control/pause.h>

bool Pause::runProc()
{
    return true;
}

void Pause::sendPause()
{
    execActionSrv.request.newActionFlag = 0;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.pause = true;
    execActionSrv.request.procType = static_cast<uint8_t>(pause__);
    execActionSrv.request.serialNum = 0;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Pause::sendUnPause()
{
    execActionSrv.request.newActionFlag = 0;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.pause = false;
    execActionSrv.request.procType = static_cast<uint8_t>(pause__);
    execActionSrv.request.serialNum = 0;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}
