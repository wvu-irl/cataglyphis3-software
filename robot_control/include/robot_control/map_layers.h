#ifndef MAP_LAYERS_H
#define MAP_LAYERS_H
#include <string>
#include <ros/ros.h>

#define NUM_MAP_LAYERS 12
#define MAP_KEYFRAME_LAYERS_START_INDEX 1
#define MAP_KEYFRAME_LAYERS_END_INDEX 3
#define MAP_SAMPLE_PROB_LAYERS_START_INDEX 4
#define MAP_SAMPLE_PROB_LAYERS_END_INDEX 9
enum MAP_LAYERS_T {_slope, _driveability, _objectHeight, _reflectivity, _cachedProb, _purpleProb, _redProb, _blueProb, _silverProb, _brassProb,
						 _roiNum, _keyframeWriteIntoGlobalMapSerialNum};
enum DRIVEABILITY_T {_noObject, _passableOverhang, _impassable};
inline std::string layerToString(MAP_LAYERS_T layer)
{
	switch(layer)
	{
	case _slope: return "slope";
	case _driveability: return "driveability";
	case _objectHeight: return "objectHeight";
	case _reflectivity: return "reflectivity";
	case _purpleProb: return "purpleProb";
	case _redProb: return "redProb";
	case _blueProb: return "blueProb";
	case _silverProb: return "silverProb";
	case _brassProb: return "brassProb";
	case _roiNum: return "roiNum";
	case _keyframeWriteIntoGlobalMapSerialNum: return "keyframeWriteIntoGlobalMapSerialNum";
	default: ROS_FATAL("layerToString: unknown layer type");
	}
}

#endif // MAP_LAYERS_H
