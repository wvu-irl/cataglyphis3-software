#ifndef MISSION_PLANNING_ENUMS_H
#define MISSION_PLANNING_ENUMS_H

#include "../../../robot_control/include/robot_control/mission_planning_types_defines.h"
#include <map>

#define QT_READ_ONLY_ROLE (Qt::UserRole+1)
#define QT_MISSION_DATA_ROLE (Qt::UserRole)

namespace mission_planning_enums
{
    enum PROC_COLUMN_BOOL_T {to_be_executed, interrupted, executing, resume};

    //std::map<

}


#endif // MISSION_PLANNING_ENUMS_H

