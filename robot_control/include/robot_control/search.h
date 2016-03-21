#ifndef SEARCH_H
#define SEARCH_H
#include "action.h"

class Search : public Action
{
public:
	void init();
	int run();
};

#endif // SEARCH_H
