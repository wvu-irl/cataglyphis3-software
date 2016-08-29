#include "mission_planning_proc_model.h"

mission_planning_proc_model::mission_planning_proc_model(QObject *parent):
    QStandardItemModel(parent)
{
    setupTable();
}

mission_planning_proc_model::mission_planning_proc_model(int row, int column, QObject *parent):
    QStandardItemModel(row, column, parent)
{
    setupTable();
}

void mission_planning_proc_model::addAllColumns()
{
    once = true;
    addProcStateColumn(&lastMissionInfo);
}

void mission_planning_proc_model::on_confirm_changes()
{

}

void mission_planning_proc_model::on_discard_changes()
{

}

void mission_planning_proc_model::on_mission_planning_info_callback(const messages::MissionPlanningInfo info)
{
    ROS_DEBUG("mission planning callback");
    lastMissionInfo = info;
    refreshTable();
}

QList<QStandardItem*> mission_planning_proc_model::addProcBoolColumn(messages::MissionPlanningInfo *info, mission_planning_enums::PROC_COLUMN_BOOL_T column)
{

}

QList<QStandardItem*> mission_planning_proc_model::addProcStateColumn(messages::MissionPlanningInfo *info)
{
    ROS_WARN("Adding new states");
    for(int row = 0; row < info->procStates.size(); row++)
    {
        QModelIndex index = this->index(row, 1, QModelIndex());
        ROS_WARN("info- procstates %d", info->procStates.at(row));
        this->setData(index, info->procStates.at(row), Qt::EditRole+1);
        //emit dataChanged(index,index);
    }
}
