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
	int roiSearchedSum;
	const int purpleProbGain = 1000;
	const int redProbGain = 1000;
	const int blueProbGain = 1000;
	const int silverProbGain = 1000;
	const int brassProbGain = 1000;
    const int distanceGain = 1000;
    const int terrainGain = 1000;
    // Methods
    bool runProc();
};

#endif // NEXT_BEST_REGION_H
