#ifndef MISSION_PLANNING_TYPES_DEFINES_H
#define MISSION_PLANNING_TYPES_DEFINES_H

#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI
#define NUM_PROC_TYPES 17
#define MAX_SAMPLES 10
#define NUM_TIMERS 5
// !!! If PROC_TYPES_T is ever edited, edit controlCallback_ in MissionPlanning as well
enum PROC_TYPES_T {__initialize__, __emergencyEscape__,__avoid__, __biasRemoval__, __nextBestRegion__, __searchRegion__, __examine__, __approach__, __collect__, __confirmCollect__, __reorient__, __goHome__, __squareUpdate__, __depositApproach__, __depositSample__, __safeMode__, __sosMode__};
enum TIMER_NAMES_T {_roiTimer_, _biasRemovalTimer_, _homingTimer_, _biasRemovalActionTimeoutTimer_, _searchTimer_};

#endif //MISSION_PLANNING_TYPES_DEFINES_H

