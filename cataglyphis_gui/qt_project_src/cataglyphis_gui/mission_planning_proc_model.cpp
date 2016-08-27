#include "mission_planning_proc_model.h"

mission_planning_proc_model::mission_planning_proc_model(QObject *parent):
    QStandardItemModel(parent)
{

}

void mission_planning_proc_model::addAllColumns()
{

}

void mission_planning_proc_model::on_confirm_changes()
{

}

void mission_planning_proc_model::on_discard_changes()
{

}

void mission_planning_proc_model::on_mission_planning_info_callback(const messages::MissionPlanningInfo info)
{

}

QList<QStandardItem*> mission_planning_proc_model::addProcBoolColumn(messages::MissionPlanningInfo *info, PROC_COLUMN_BOOL_T column)
{

}

QList<QStandardItem*> mission_planning_proc_model::addProcStateColumn(messages::MissionPlanningInfo *info)
{

}
