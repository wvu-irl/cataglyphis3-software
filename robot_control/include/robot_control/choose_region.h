#ifndef CHOOSE_REGION_H
#define CHOOSE_REGION_H
#include "process.h"

class ChooseRegion : public Process
{
public:
    // Members
    int bestROINum;
    int roiValue;
    int bestROIValue;
    int roiVisitedSum;
    const int easyProbGain = 1000;
    const int medProbGain = 1000;
    const int hardProbGain = 1000;
    const int distanceGain = 1000;
    const int terrainGain = 1000;
    // Methods
    bool runProc();
};

#endif // CHOOSE_REGION_H
