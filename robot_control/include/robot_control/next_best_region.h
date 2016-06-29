#ifndef NEXT_BEST_REGION_H
#define NEXT_BEST_REGION_H
#include "procedure.h"

class NextBestRegion : public Procedure
{
public:
    // Members
    int bestROINum;
	float roiValue;
	float bestROIValue;
	int roiSearchedSum;
	const float sampleProbGain = 1.0;
	const float distanceGain = 1.0;
	const float terrainGain = 1.0;
    // Methods
    bool runProc();
};

#endif // NEXT_BEST_REGION_H
