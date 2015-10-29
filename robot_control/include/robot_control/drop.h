#ifndef DROP_H
#define DROP_H
#include "action.h"

class Drop : public Action
{
public:
	void init();
	int run();
};

#endif // DROP_H
