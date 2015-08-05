#ifndef ACTION_H
#define ACTION_H

enum ACTION_TYPE_T {_stop, _travelWP, _drive, _pivot, _grab, _drop, _open, _search, _approach, _confirmCollect, _reorient, _deposit};

class Action
{
public:
	// Members

	// Methods
	Action(ACTION_TYPE_T actionType);
	Action(ACTION_TYPE_T actionType, float param);
	Action(ACTION_TYPE_T actionType, int param);
	Action(ACTION_TYPE_T actionType, float param1, float param2);
	int run();
};

#endif // ACTION_H
