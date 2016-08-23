#ifndef EXEC_ACTION_ENUMS
#define EXEC_ACTION_ENUMS

#include "../../../robot_control/include/robot_control/action_type_enum.h"
//#include <map>

namespace exec_action_enums
{
    static std::map<ACTION_TYPE_T, std::string> actionTypeToString {{_idle, "Idle"},
                                                                    {_halt, "Halt"},
                                                                    {_driveGlobal, "Drive Global"},
                                                                    {_driveRelative, "Drive Relative"},
                                                                    {_grab, "Grab"},
                                                                    {_drop, "Drop"},
                                                                    {_open, "Open"},
                                                                    {_search, "Search"},
                                                                    {_wait, "Wait"}};
}

#endif // EXEC_ACTION_ENUMS

