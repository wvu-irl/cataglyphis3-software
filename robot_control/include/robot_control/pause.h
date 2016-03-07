#ifndef PAUSE_H
#define PAUSE_H
#include "process.h"

class Pause : public Process
{
public:
    // Methods
    void sendPause();
    void sendUnPause();
    bool runProc();
};

#endif // PAUSE_H
