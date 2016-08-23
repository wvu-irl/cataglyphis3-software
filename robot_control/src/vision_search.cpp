#include <robot_control/vision_search.h>

void VisionSearch::init()
{
    ROS_INFO("init vision_search");
    cvSearchCmdSrv.request.white = params.float1;
    ROS_INFO("white = %f",cvSearchCmdSrv.request.white);
    cvSearchCmdSrv.request.silver = params.float2;
    cvSearchCmdSrv.request.blueOrPurple = params.float3;
    cvSearchCmdSrv.request.pink = params.float4;
    cvSearchCmdSrv.request.red = params.float5;
    cvSearchCmdSrv.request.orange = params.float6;
    cvSearchCmdSrv.request.yellow = params.float7;
    cvSearchCmdSrv.request.save = false;
    cvSearchCmdSrv.request.live = true;
	cvSearchCmdSrv.request.procType = params.procType;
	cvSearchCmdSrv.request.serialNum = params.serialNum;
}

int VisionSearch::run()
{
    if(cvSearchCmdClient.call(cvSearchCmdSrv)) ROS_DEBUG("cvSearchCmdClient call successful");
    else {ROS_ERROR("cvSearchCmdClient call unsuccessful");}
    return 1;
}
