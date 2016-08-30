#include <robot_control/search.h>

void Search::init()
{
	clearDeques();
    dropFailed_ = false;
    slidesFailed_ = false;
	pushTask(_search_);
    visionDeque.back()->params.float1 = params.float1;
    visionDeque.back()->params.float2 = params.float2;
    visionDeque.back()->params.float3 = params.float3;
    visionDeque.back()->params.float4 = params.float4;
    visionDeque.back()->params.float5 = params.float5;
    visionDeque.back()->params.float6 = params.float6;
    visionDeque.back()->params.float7 = params.float7;
	visionDeque.back()->params.procType = params.procType;
	visionDeque.back()->params.serialNum = params.serialNum;
}

int Search::run()
{
	return runDeques();
}
