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

#ifndef MAP_VIEWER_ENUMS_H
#define MAP_VIEWER_ENUMS_H

#include "../../../robot_control/include/robot_control/map_layers.h"
#include "../../../robot_control/include/robot_control/task_type_enum.h"

namespace map_viewer_enums
{
    typedef MAP_LAYERS_T GRID_MAP_LAYERS_T;

//for telling a scene which layer to activate/deactivate
    enum mapViewerLayers_t {keyframeDrive, ROI, PATH, GRIDMAP, SatDriveability};
    static std::map<mapViewerLayers_t, std::string> mapViewerLayersToString {{keyframeDrive,   "Keyframe Driveability"},
                                                                             {ROI,    "ROI"},
                                                                             {PATH,   "Path"},
                                                                             {GRIDMAP, "Grid Map"},
                                                                             {SatDriveability, "Satellite Driveability"},
                                                                            };

    static std::map<GRID_MAP_LAYERS_T, std::string> gridMapLayersToString   {{_satDriveability, "satDriveability"},
                                                                             {_keyframeDriveability, "keyframeDriveability"},
                                                                            };

    static std::map<mapViewerLayers_t, GRID_MAP_LAYERS_T> mapViewerLayersToGridMapLayers {{keyframeDrive, _keyframeDriveability},
                                                                                          {SatDriveability, _satDriveability}};

}


#endif // MAP_VIEWER_ENUMS_H
