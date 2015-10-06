#ifndef IDLE_H
#define IDLE_H
#include "action.h"

class Idle : public Action
{
public:
    void init();
    int run();
};

#endif // IDLE_H
