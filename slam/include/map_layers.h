/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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
