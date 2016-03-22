#ifndef NEXT_BEST_REGION_H
#define NEXT_BEST_REGION_H
#include "procedure.h"

class NextBestRegion : public Procedure
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

#endif // NEXT_BEST_REGION_H
