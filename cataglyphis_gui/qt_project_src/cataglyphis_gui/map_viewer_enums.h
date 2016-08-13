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

