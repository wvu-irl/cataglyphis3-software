#ifndef MAP_LAYERS_H
#define MAP_LAYERS_H
#include <string>

#define NUM_MAP_LAYERS 12
#define MAP_KEYFRAME_LAYERS_START_INDEX 1
#define MAP_KEYFRAME_LAYERS_END_INDEX 5
#define MAP_SAMPLE_PROB_LAYERS_START_INDEX 6
#define MAP_SAMPLE_PROB_LAYERS_END_INDEX 10
enum MAP_LAYERS_T {_slope, _driveability, _objectType, _objectIndex, _objectHeight, _reflectivity, _purpleProb, _redProb, _blueProb, _silverProb, _brassProb,
						 _roiNum};
inline std::string layerToString(MAP_LAYERS_T layer)
{
	switch(layer)
	{
	case _slope: return "slope";
	case _driveability: return "driveability";
	case _objectType: return "objectType";
	case _objectIndex: return "objectIndex";
	case _objectHeight: return "objectHeight";
	case _reflectivity: return "reflectivity";
	case _purpleProb: return "purpleProb";
	case _redProb: return "redProb";
	case _blueProb: return "blueProb";
	case _silverProb: return "silverProb";
	case _brassProb: return "brassProb";
	}
}

#endif // MAP_LAYERS_H
