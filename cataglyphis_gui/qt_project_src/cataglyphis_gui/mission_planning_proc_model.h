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
