#include <robot_control/search.h>

void Search::init()
{
	clearDeques();
	pushTask(_search_);
	visionDeque.back()->params.bool1 = params.bool1;
	visionDeque.back()->params.bool2 = params.bool2;
	visionDeque.back()->params.bool3 = params.bool3;
	visionDeque.back()->params.bool4 = params.bool4;
	visionDeque.back()->params.bool5 = params.bool5;
	visionDeque.back()->params.bool6 = params.bool6;
	visionDeque.back()->params.bool7 = params.bool7;
	visionDeque.back()->params.procType = params.procType;
	visionDeque.back()->params.serialNum = params.serialNum;
}

int Search::run()
{
	return runDeques();
}
