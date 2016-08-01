#ifndef SQUARE_ROIS_H
#define SQUARE_ROIS_H

ROI.e = 7.0;
ROI.s = 7.0;
ROI.sampleProb = 1.0;
ROI.sampleSig = 10.0;
ROI.radialAxis = 15.0;
ROI.tangentialAxis = 10.0;
ROI.searched = false;
regionsOfInterest.push_back(ROI);
ROI.e = -7.0;
ROI.s = 7.0;
ROI.sampleProb = 0.5;
ROI.sampleSig = 10.0;
ROI.radialAxis = 15.0;
ROI.tangentialAxis = 10.0;
ROI.searched = false;
regionsOfInterest.push_back(ROI);
ROI.e = -7.0;
ROI.s = -7.0;
ROI.sampleProb = 0.4;
ROI.sampleSig = 10.0;
ROI.radialAxis = 15.0;
ROI.tangentialAxis = 10.0;
ROI.searched = false;
regionsOfInterest.push_back(ROI);
ROI.e = 7.0;
ROI.s = -7.0;
ROI.sampleProb = 0.3;
ROI.sampleSig = 10.0;
ROI.radialAxis = 15.0;
ROI.tangentialAxis = 10.0;
ROI.searched = false;
regionsOfInterest.push_back(ROI);

#endif // SQUARE_ROIS_H
