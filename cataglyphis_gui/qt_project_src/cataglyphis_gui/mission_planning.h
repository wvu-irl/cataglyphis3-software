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

#ifndef MISSION_PLANNING_H
#define MISSION_PLANNING_H

#include <ros/ros.h>

#include <QWidget>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTableView>
#include <qledindicator.h>
#include <QThread>

#include <qcomboboxdelegate.h>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <messages/MissionPlanningInfo.h>
#include <messages/MissionPlanningControl.h>
#include <mission_planning_proc_model.h>

#include <ros_workers.h>

#include <QTime>

#define BOOL_CHECKER
#define I_AM_DEFINED
#define MOVE_DATA_1_2(arg1, arg2) {arg1 = arg2;}
#define MOVE_DATA_2_1(arg1, arg2) {arg2 = arg1;}

//boolDir true for setting arg1 = arg2
#define DIRECTION_MOVE(arg2, arg1, boolDir) { \
        if(boolDir) \
            MOVE_DATA_1_2(arg1, arg2->isChecked()) \
        else\
            arg2->setChecked(arg1); \
        } \

#define DIRECTION_MOVE_SPINBOX(arg2, arg1, boolDir) { \
        if(boolDir) \
            MOVE_DATA_1_2(arg1, arg2->value()) \
        else \
            arg2->setValue(arg1); \
    } \

#define DIRECTION_MOVE_NORMAL(arg2, arg1, boolDir) { \
        if(boolDir) \
            MOVE_DATA_1_2(arg1, arg2) \
        else \
            MOVE_DATA_2_1(arg1, arg2); \
    } \


namespace Ui {
class mission_planning_form;
}

class mission_planning : public QWidget
{
    Q_OBJECT

signals:
    void update_mission_timer(double time);
    void add_wait_to_exec(float seconds);
    void modify_mission_planning_request(messages::MissionPlanningControl info);
    void start_mission_planning_callback();
    void stop_mission_planning_callback();

public slots:
    void on_mission_planning_info_callback(const messages::MissionPlanningInfo info);
    void on_confirm_changes();
    void on_discard_changes();

public:
    explicit mission_planning(QWidget *parent = 0);
    ~mission_planning();

    bool isModified(){return modified;}
    void connectSignals();

private slots:

    void on_reset_edit_button_clicked();

    void on_edit_mission_planning_button_clicked(bool checked);

    void on_commit_changes_button_clicked();

private:
    Ui::mission_planning_form *ui;
    mission_planning_proc_model procTabelModel;
    boost::scoped_ptr<ComboBoxDelegate> procDelegate;
    boost::scoped_ptr<ComboBoxDelegate> executeDelegate;
    boost::scoped_ptr<ComboBoxDelegate> interruptDelegate;
    boost::scoped_ptr<ComboBoxDelegate> beginExecutedDelegate;
    boost::scoped_ptr<ComboBoxDelegate> resumeDelegate;

    QThread rosWorkerThread;
    boost::shared_ptr<ros_workers> rosWorker;
    boost::shared_ptr<ros::NodeHandle> nh;

    //uiToMsg true if moving ui data to the message
    void _implMoveData(messages::MissionPlanningInfo *info, bool uiToMsg);
    void _implMoveUiToService(messages::MissionPlanningControl *serviceInfo);
    messages::MissionPlanningInfo * _implCurrentDataSet();
    void _implSetReadOnly(bool readOnly);

    bool modified;
    messages::MissionPlanningInfo latestInfo;
    messages::MissionPlanningInfo stagedInfo;

    boost::scoped_ptr<mission_planning_proc_model> tableModel;

    double missionTime;

};

#endif // MISSION_PLANNING_H

//bool performReorient
//uint32 reorientCount
