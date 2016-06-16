#include <robot_control/search_region.h>

bool SearchRegion::runProc()
{
	ROS_INFO("searchRegion state = %i", state);
	switch(state)
	{
	case _init_:
		avoidLockout = false;
		procsBeingExecuted[procType] = true;
		procsToExecute[procType] = false;
		execDequeEmpty = false;
		// check if ROI is not yet keyframed
		if(!roiKeyframed)
		{
			searchMapSrv.request.createMap = true;
			searchMapSrv.request.roiIndex = currentROIIndex;
			searchMapSrv.request.deleteMap = false;
			if(searchMapClient.call(searchMapSrv)) ROS_DEBUG("searchMap service call successful");
			else ROS_ERROR("searchMap service call unsuccessful");
		}
		// start timer based on elapsed time
		break;
	case _exec_:

		break;
	case _interrupt_:

		break;
	case _finish_:

		break;
	}
}
