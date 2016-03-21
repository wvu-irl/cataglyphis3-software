#include <robot_control/vision_search.h>

void VisionSearch::init()
{
	cvSearchCmdSrv.request.purple = params.bool1;
	cvSearchCmdSrv.request.red = params.bool2;
	cvSearchCmdSrv.request.blue = params.bool3;
	cvSearchCmdSrv.request.silver = params.bool4;
	cvSearchCmdSrv.request.brass = params.bool5;
	cvSearchCmdSrv.request.confirm = params.bool6;
	cvSearchCmdSrv.request.save = params.bool7;
	cvSearchCmdSrv.request.procType = params.procType;
	cvSearchCmdSrv.request.serialNum = params.serialNum;
}

int VisionSearch::run()
{
	if(cvSearchCmdClient.call(cvSearchCmdSrv)) {ROS_DEBUG("cvSearchCmdClient call successful"); return 1;}
	else {ROS_ERROR("cvSearchCmdClient call unsuccessful"); return 0;}
}
