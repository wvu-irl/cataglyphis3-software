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
