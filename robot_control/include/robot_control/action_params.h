#ifndef ACTION_PARAMS_H
#define ACTION_PARAMS_H
#include "action_type_enum.h"
#include "mission_planning_procedure_share.h"

struct ACTION_PARAMS_T
{
	ACTION_TYPE_T actionType;
	float float1;
	float float2;
	float float3;
	float float4;
	float float5;
	float float6;
	float float7;
	int int1;
	bool bool1;
	bool bool2;
	bool bool3;
	bool bool4;
	bool bool5;
	bool bool6;
	bool bool7;
	bool bool8;
	PROC_TYPES_T procType;
	unsigned int serialNum;
};

#endif // ACTION_PARAMS_H
