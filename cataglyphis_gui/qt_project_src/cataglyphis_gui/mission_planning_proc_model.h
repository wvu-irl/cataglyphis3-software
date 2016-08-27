#ifndef MISSION_PLANNING_PROC_MODEL_H
#define MISSION_PLANNING_PROC_MODEL_H

#include <ros/ros.h>

#include <QObject>
#include <QStandardItemModel>
#include <QThread>
#include <QList>
#include <QStringList>
#include <ros_workers.h>

#include <messages/MissionPlanningInfo.h>
#include <messages/MissionPlanningControl.h>

class mission_planning_proc_model : public QStandardItemModel
{
signals:
    void start_mission_planning_info_callback();
    void stop_mission_planning_info_callback();

public slots:
    void on_mission_planning_info_callback(const messages::MissionPlanningInfo info);
    void on_confirm_changes();
    void on_discard_changes();

public:

    enum PROC_COLUMN_BOOL_T {resume, executing, interrupted, to_be_executed};

    mission_planning_proc_model(QObject *parent = 0);

    QList<QStandardItem*> addProcStateColumn(messages::MissionPlanningInfo *info);
    QList<QStandardItem*> addProcBoolColumn(messages::MissionPlanningInfo *info, PROC_COLUMN_BOOL_T column);

    void addAllColumns();

    void setupTable()
    {
        setHorizontalHeaderLabels(horizontalLabels);
        setVerticalHeaderLabels(verticalLabels);
    }
    void clearTable()
    {
        this->clear();
        itemList.clear();
    }
    void refreshTable()
    {
        clearTable();
        addAllColumns();
    }

private:

    messages::MissionPlanningInfo lastMissionInfo;

    QList<QStandardItem> itemList;
    QStringList horizontalLabels;
    QStringList verticalLabels;
};

#endif // MISSION_PLANNING_PROC_MODEL_H
