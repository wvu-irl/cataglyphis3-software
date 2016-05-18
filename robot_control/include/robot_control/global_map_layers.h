#ifndef GLOBAL_MAP_LAYERS_H
#define GLOBAL_MAP_LAYERS_H
#include <string>

#define NUM_GLOBAL_MAP_LAYERS 11
enum GLOBAL_MAP_LAYERS_T {_slope, _driveability, _objectType, _objectIndex, _objectHeight, _reflectivity, _purpleProb, _redProb, _blueProb, _silverProb, _brassProb};
inline std::string layerToString(GLOBAL_MAP_LAYERS_T layer)
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

#endif // GLOBAL_MAP_LAYERS_H
