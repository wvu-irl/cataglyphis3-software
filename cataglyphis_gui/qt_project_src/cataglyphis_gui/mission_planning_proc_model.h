#ifndef MISSION_PLANNING_PROC_MODEL_H
#define MISSION_PLANNING_PROC_MODEL_H

#include <ros/ros.h>

#include <QObject>
#include <QStandardItemModel>
#include <QThread>
#include <QList>
#include <QStringList>
#include <ros_workers.h>
#include <QModelIndex>

#include <mission_planning_enums.h>

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

    mission_planning_proc_model(QObject *parent = 0);
    mission_planning_proc_model(int row, int column, QObject *parent = 0);

    void addProcStateColumn(messages::MissionPlanningInfo *info);
    void addProcBoolColumn(std::vector<uint8_t> *data, int column);

    std::vector<uint8_t> formBoolVectorFromColumn(int column);
    std::vector<int32_t> formIntVectorFromColumn(int column);

    void addAllColumns();

    void setReadOnly(bool readOnly);

    void setupTable()
    {
        verticalLabels.append("Initialize");
        verticalLabels.append("Emergency\nEscape");
        verticalLabels.append("Avoid");
        verticalLabels.append("Bias\nRemoval");
        verticalLabels.append("Next Best\nRegion");
        verticalLabels.append("Search\nRegion");
        verticalLabels.append("Examine");
        verticalLabels.append("Approach");
        verticalLabels.append("Collect");
        verticalLabels.append("Confirm\nCollect");
        verticalLabels.append("Re-Orient");
        verticalLabels.append("Go Home");
        verticalLabels.append("Square\nUpdate");
        verticalLabels.append("Deposit\nApproach");
        verticalLabels.append("Deposit\nSample");
        verticalLabels.append("Safe Mode");
        verticalLabels.append("SOS Mode");
        horizontalLabels.append("State");
        horizontalLabels.append("To\nExecute");
        horizontalLabels.append("To\nInterrupt");
        horizontalLabels.append("Being\nExecuted");
        horizontalLabels.append("To\nResume");

        setHorizontalHeaderLabels(horizontalLabels);
        setVerticalHeaderLabels(verticalLabels);
    }
    void clearTable()
    {
        //this->clear();
        itemList.clear();

    }
    void refreshTable()
    {
        clearTable();
        addAllColumns();
    }

private:

    messages::MissionPlanningInfo lastMissionInfo;

    bool once;
    QList<QStandardItem> itemList;
    QStringList horizontalLabels;
    QStringList verticalLabels;
};

#endif // MISSION_PLANNING_PROC_MODEL_H
