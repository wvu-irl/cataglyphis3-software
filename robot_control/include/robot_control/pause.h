#ifndef PAUSE_H
#define PAUSE_H
#include "procedure.h"

class Pause : public Procedure
{
public:
    // Methods
    void sendPause();
    void sendUnPause();
    bool runProc();
};

#endif // PAUSE_H
