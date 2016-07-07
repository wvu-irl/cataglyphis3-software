#include <robot_control/vision_search.h>

void VisionSearch::init()
{
    cvSearchCmdSrv.request.cached = params.bool1;
    cvSearchCmdSrv.request.purple = params.bool2;
    cvSearchCmdSrv.request.red = params.bool3;
    cvSearchCmdSrv.request.blue = params.bool4;
    cvSearchCmdSrv.request.silver = params.bool5;
    cvSearchCmdSrv.request.brass = params.bool6;
    cvSearchCmdSrv.request.confirm = params.bool7;
    cvSearchCmdSrv.request.save = params.bool8;
	cvSearchCmdSrv.request.procType = params.procType;
	cvSearchCmdSrv.request.serialNum = params.serialNum;
}

int VisionSearch::run()
{
	if(cvSearchCmdClient.call(cvSearchCmdSrv)) {ROS_DEBUG("cvSearchCmdClient call successful"); return 1;}
	else {ROS_ERROR("cvSearchCmdClient call unsuccessful"); return 0;}
}
