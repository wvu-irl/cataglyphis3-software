#ifndef MAP_LAYERS_H
#define MAP_LAYERS_H
#include <string>
#include <ros/ros.h>

#define NUM_MAP_LAYERS 12
#define MAP_SAT_LAYERS_START_INDEX 1
#define MAP_SAT_LAYERS_END_INDEX 3
#define MAP_KEYFRAME_LAYERS_START_INDEX 4
#define MAP_KEYFRAME_LAYERS_END_INDEX 7
enum MAP_LAYERS_T {_slope, _satDriveability, _satDriveabilityConf, _satObjectHeight, _keyframeDriveability, _keyframeDriveabilityConf, _keyframeObjectHeight,
				   _reflectivity, _sampleProb, _localMapDriveability, _roiNum, _keyframeWriteIntoGlobalMapSerialNum};
inline std::string layerToString(MAP_LAYERS_T layer)
{
	switch(layer)
	{
	case _slope: return "slope";
	case _satDriveability: return "satDriveability";
	case _satDriveabilityConf: return "satDriveabilityConf";
	case _satObjectHeight: return "satObjectHeight";
	case _keyframeDriveability: return "keyframeDriveability";
	case _keyframeDriveabilityConf: return "keyframeDriveabilityConf";
	case _keyframeObjectHeight: return "keyframeObjectHeight";
	case _reflectivity: return "reflectivity";
	case _localMapDriveability: return "localMapDriveability";
	case _sampleProb: return "sampleProb";
	case _roiNum: return "roiNum";
	case _keyframeWriteIntoGlobalMapSerialNum: return "keyframeWriteIntoGlobalMapSerialNum";
	default: ROS_FATAL("layerToString: unknown layer type");
	}
}

#endif // MAP_LAYERS_H
