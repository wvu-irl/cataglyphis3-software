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

#include "mission_planning.h"
#include "ui_mission_planning_form.h"

mission_planning::mission_planning(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::mission_planning_form)
{
    ui->setupUi(this);
    rosWorker = boost::shared_ptr<ros_workers>(new ros_workers());
    rosWorker->moveToThread(&rosWorkerThread);
    rosWorkerThread.start();

    connectSignals();

    modified = false;

    tableModel.reset(new mission_planning_proc_model(17,5));
    ui->proc_table->setModel(tableModel.get());
    QList<QString> tempList;
    tempList.append("Init");tempList.append("Exec");tempList.append("Interrupt");tempList.append("Finish");
    procDelegate.reset(new ComboBoxDelegate(tempList));
    tempList.clear();
    tempList.append("False");tempList.append("True");
    executeDelegate.reset(new ComboBoxDelegate(tempList));
    interruptDelegate.reset(new ComboBoxDelegate(tempList));
    beginExecutedDelegate.reset(new ComboBoxDelegate(tempList));
    resumeDelegate.reset(new ComboBoxDelegate(tempList));
    ui->proc_table->setItemDelegateForColumn(0, procDelegate.get());
    ui->proc_table->setItemDelegateForColumn(static_cast<int>(mission_planning_enums::executing)+1, executeDelegate.get());
    ui->proc_table->setItemDelegateForColumn(static_cast<int>(mission_planning_enums::interrupted)+1, interruptDelegate.get());
    ui->proc_table->setItemDelegateForColumn(static_cast<int>(mission_planning_enums::to_be_executed)+1, beginExecutedDelegate.get());
    ui->proc_table->setItemDelegateForColumn(static_cast<int>(mission_planning_enums::resume)+1, resumeDelegate.get());
    ui->proc_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    for(int row = 0; row < 17; row++)
    {
        ROS_DEBUG("Setting default data");
        for(int column = 0; column < 5; column ++)
        {
            QModelIndex index = tableModel->index(row, column, QModelIndex());
            tableModel->setData(index, 0, QT_MISSION_DATA_ROLE);
            ui->proc_table->openPersistentEditor(index);
        }
    }

    missionTime = 0;

    _implSetReadOnly(true);

    emit start_mission_planning_callback();
}

mission_planning::~mission_planning()
{
    std::printf("Mission Planning Destruction\r\n");
    rosWorkerThread.quit();
    rosWorkerThread.wait();
    delete ui;
}

messages::MissionPlanningInfo * mission_planning::_implCurrentDataSet()
{
    if(isModified())
    {
        ROS_DEBUG("MissionPlanning:: Current data set is: staged");
        return &stagedInfo;
    }
    ROS_DEBUG("MissionPlanning:: Current data set is: latest");
    return &latestInfo;
}

void mission_planning::on_mission_planning_info_callback(const messages::MissionPlanningInfo info)
{
    ROS_DEBUG_THROTTLE(5, "Mission Planning Callback %d", (int)isModified());
    latestInfo = info;
    if(!isModified())
    {
        ROS_DEBUG_THROTTLE(5, "Updating display");
        _implMoveData(&latestInfo, false);
        tableModel->on_mission_planning_info_callback(info);
    }
    emit update_mission_timer(info.missionTime);
}

void mission_planning::on_commit_changes_button_clicked()
{
    if(isModified())
    {
        ROS_DEBUG("Mission Planning Modified!");
        on_confirm_changes();
    }
}

void mission_planning::on_confirm_changes()
{
    ROS_WARN("Commiting Changes to Mission Planning!");
    messages::MissionPlanningControl info;
    _implMoveUiToService(&info);
    modified = false;

    emit modify_mission_planning_request(info);
    emit add_wait_to_exec(1); //add 1 second wait to exec

    on_reset_edit_button_clicked();
}

void mission_planning::on_discard_changes()
{
    modified = false;
    ui->edit_mission_planning_button->setChecked(false);
    _implSetReadOnly(true);
    _implMoveData(_implCurrentDataSet(), true);
}

void mission_planning::on_reset_edit_button_clicked()
{
    on_discard_changes();
}

void mission_planning::on_edit_mission_planning_button_clicked(bool checked)
{
    if(checked)
    {
        ROS_DEBUG("Mission Planning:: Edit button clicked %d", (int)checked);
        modified = checked;
        _implSetReadOnly(!checked);
    }
    ui->edit_mission_planning_button->setChecked(true);
}

void mission_planning::connectSignals()
{
    connect(this, &mission_planning::add_wait_to_exec,
                rosWorker.get(), &ros_workers::on_add_pause_to_exec_queue);
    connect(this, &mission_planning::modify_mission_planning_request,
                rosWorker.get(), &ros_workers::on_run_mission_planning_service);

    connect(rosWorker.get(), &ros_workers::mission_planning_info_callback,
                this, &mission_planning::on_mission_planning_info_callback);

    connect(this, &mission_planning::start_mission_planning_callback,
                rosWorker.get(), &ros_workers::on_run_mission_planning_info_subscriber_start);
    connect(this, &mission_planning::stop_mission_planning_callback,
                rosWorker.get(), &ros_workers::on_run_mission_planning_info_subscriber_stop);
}

//uiToMsg true if moving ui data to the message
void mission_planning::_implMoveData(messages::MissionPlanningInfo *info, bool uiToMsg)
{
            DIRECTION_MOVE(ui->at_home_indicator,                   info->atHome, uiToMsg);
    DIRECTION_MOVE_SPINBOX(ui->avoid_count_spinbox,                 info->avoidCount, uiToMsg);
            DIRECTION_MOVE(ui->avoid_lockout_indicator,             info->avoidLockout, uiToMsg);
    DIRECTION_MOVE_SPINBOX(ui->backup_count_spinbox,                info->backupCount, uiToMsg);
    DIRECTION_MOVE_SPINBOX(ui->collision_condition_spinbox,         info->collisionCondition, uiToMsg);
    DIRECTION_MOVE_SPINBOX(ui->confirm_collect_failed_count_spinbox,info->confirmCollectFailedCount, uiToMsg);
            DIRECTION_MOVE(ui->escape_condition_indicator,          info->escapeCondition, uiToMsg);
            DIRECTION_MOVE(ui->escape_lockout_indicator,            info->escapeLockout, uiToMsg);
    DIRECTION_MOVE_SPINBOX(ui->examine_count_spinbox,               info->examineCount, uiToMsg);
            DIRECTION_MOVE(ui->execute_avoid_indicator,             info->shouldExecuteAvoidManeuver, uiToMsg);
            DIRECTION_MOVE(ui->give_up_roi_indicator,               info->giveUpROI, uiToMsg);
            DIRECTION_MOVE(ui->homing_update_failed_indicator,      info->homingUpdateFailed, uiToMsg);
    DIRECTION_MOVE_SPINBOX(ui->homin_update_failed_count_spinbox,   info->homingUpdatedFailedCount, uiToMsg);
            DIRECTION_MOVE(ui->initialized_indicator,               info->initialized, uiToMsg);
            DIRECTION_MOVE(ui->in_deposit_pos_indicator,            info->inDepositPosition, uiToMsg);
            DIRECTION_MOVE(ui->in_safe_mode_indicator,              info->performSafeMode, uiToMsg);
            DIRECTION_MOVE(ui->in_search_region_indicator,          info->inSearchableRegion, uiToMsg);
            DIRECTION_MOVE(ui->mission_ended_indicator,             info->missionEnded, uiToMsg);
    DIRECTION_MOVE_SPINBOX(ui->num_procs_spinbox,                   info->numProcs, uiToMsg);
            DIRECTION_MOVE(ui->pause_indicator,                     info->pause, uiToMsg);
            DIRECTION_MOVE(ui->perform_bias_removal_indicator,      info->performBiasRemoval, uiToMsg);
            DIRECTION_MOVE(ui->perform_homing_indicator,            info->performHoming, uiToMsg);
            DIRECTION_MOVE(ui->possessing_sample_indicator,         info->possessingSample, uiToMsg);
            DIRECTION_MOVE(ui->possession_confirmed_indicator,      info->confirmedPossession, uiToMsg);
            DIRECTION_MOVE(ui->possible_sample_sighted_indicator,   info->possibleSample, uiToMsg);
            DIRECTION_MOVE(ui->possibly_lost_indicator,             info->possiblyLost, uiToMsg);
            DIRECTION_MOVE(ui->roi_keyframe_indicator,              info->roiKeyframed, uiToMsg);
            DIRECTION_MOVE(ui->roi_time_expired_indicator,          info->roiTimeExpired, uiToMsg);
    DIRECTION_MOVE_SPINBOX(ui->sample_collected_spinbox,            info->samplesCollected, uiToMsg);
            DIRECTION_MOVE(ui->sample_in_collect_pos_indicator,     info->sampleInCollectPosition, uiToMsg);
            DIRECTION_MOVE(ui->sample_sighted_indicator,            info->definiteSample, uiToMsg);
            DIRECTION_MOVE(ui->side_grab_indicator,                 info->sideOffsetGrab, uiToMsg);
            DIRECTION_MOVE(ui->start_slam_indicator,                info->startSLAM, uiToMsg);
            DIRECTION_MOVE(ui->use_dead_reckoning_indicator,        info->useDeadReckoning, uiToMsg);
            DIRECTION_MOVE_NORMAL(tableModel->formIntVectorFromColumn(0), info->procStates, uiToMsg);
            DIRECTION_MOVE_NORMAL(tableModel->formBoolVectorFromColumn(mission_planning_enums::executing+1), info->procsBeingExecuted, uiToMsg);
            DIRECTION_MOVE_NORMAL(tableModel->formBoolVectorFromColumn(mission_planning_enums::to_be_executed+1),info->procsToExecute,  uiToMsg);
            DIRECTION_MOVE_NORMAL(tableModel->formBoolVectorFromColumn(mission_planning_enums::interrupted+1), info->procsToInterrupt, uiToMsg);
            DIRECTION_MOVE_NORMAL(tableModel->formBoolVectorFromColumn(mission_planning_enums::resume+1), info->procsToResume, uiToMsg);
            DIRECTION_MOVE_NORMAL(missionTime,                     info->missionTime, uiToMsg);
}

void mission_planning::_implMoveUiToService(messages::MissionPlanningControl *serviceInfo)
{
    DIRECTION_MOVE(ui->at_home_indicator,                   serviceInfo->request.atHome, true);
DIRECTION_MOVE_SPINBOX(ui->avoid_count_spinbox,             serviceInfo->request.avoidCount, true);
    DIRECTION_MOVE(ui->avoid_lockout_indicator,             serviceInfo->request.avoidLockout, true);
DIRECTION_MOVE_SPINBOX(ui->backup_count_spinbox,            serviceInfo->request.backupCount, true);
DIRECTION_MOVE_SPINBOX(ui->collision_condition_spinbox,     serviceInfo->request.collisionCondition, true);
DIRECTION_MOVE_SPINBOX(ui->confirm_collect_failed_count_spinbox,serviceInfo->request.confirmCollectFailedCount, true);
    DIRECTION_MOVE(ui->escape_condition_indicator,          serviceInfo->request.escapeCondition, true);
    DIRECTION_MOVE(ui->escape_lockout_indicator,            serviceInfo->request.escapeLockout, true);
DIRECTION_MOVE_SPINBOX(ui->examine_count_spinbox,           serviceInfo->request.examineCount, true);
    DIRECTION_MOVE(ui->execute_avoid_indicator,             serviceInfo->request.shouldExecuteAvoidManeuver, true);
    DIRECTION_MOVE(ui->give_up_roi_indicator,               serviceInfo->request.giveUpROI, true);
    DIRECTION_MOVE(ui->homing_update_failed_indicator,      serviceInfo->request.homingUpdateFailed, true);
DIRECTION_MOVE_SPINBOX(ui->homin_update_failed_count_spinbox,serviceInfo->request.homingUpdatedFailedCount, true);
    DIRECTION_MOVE(ui->initialized_indicator,               serviceInfo->request.initialized, true);
    DIRECTION_MOVE(ui->in_deposit_pos_indicator,            serviceInfo->request.inDepositPosition, true);
    DIRECTION_MOVE(ui->in_safe_mode_indicator,              serviceInfo->request.performSafeMode, true);
    DIRECTION_MOVE(ui->in_search_region_indicator,          serviceInfo->request.inSearchableRegion, true);
    DIRECTION_MOVE(ui->mission_ended_indicator,             serviceInfo->request.missionEnded, true);
DIRECTION_MOVE_SPINBOX(ui->num_procs_spinbox,               serviceInfo->request.numProcs, true);
    DIRECTION_MOVE(ui->pause_indicator,                     serviceInfo->request.pause, true);
    DIRECTION_MOVE(ui->perform_bias_removal_indicator,      serviceInfo->request.performBiasRemoval, true);
    DIRECTION_MOVE(ui->perform_homing_indicator,            serviceInfo->request.performHoming, true);
    DIRECTION_MOVE(ui->possessing_sample_indicator,         serviceInfo->request.possessingSample, true);
    DIRECTION_MOVE(ui->possession_confirmed_indicator,      serviceInfo->request.confirmedPossession, true);
    DIRECTION_MOVE(ui->possible_sample_sighted_indicator,   serviceInfo->request.possibleSample, true);
    DIRECTION_MOVE(ui->possibly_lost_indicator,             serviceInfo->request.possiblyLost, true);
    DIRECTION_MOVE(ui->roi_keyframe_indicator,              serviceInfo->request.roiKeyframed, true);
    DIRECTION_MOVE(ui->roi_time_expired_indicator,          serviceInfo->request.roiTimeExpired, true);
DIRECTION_MOVE_SPINBOX(ui->sample_collected_spinbox,        serviceInfo->request.samplesCollected, true);
    DIRECTION_MOVE(ui->sample_in_collect_pos_indicator,     serviceInfo->request.sampleInCollectPosition, true);
    DIRECTION_MOVE(ui->sample_sighted_indicator,            serviceInfo->request.definiteSample, true);
    DIRECTION_MOVE(ui->side_grab_indicator,                 serviceInfo->request.sideOffsetGrab, true);
    DIRECTION_MOVE(ui->start_slam_indicator,                serviceInfo->request.startSLAM, true);
    DIRECTION_MOVE(ui->use_dead_reckoning_indicator,        serviceInfo->request.useDeadReckoning, true);
    DIRECTION_MOVE_NORMAL(missionTime,                     serviceInfo->request.missionTime, true);
    serviceInfo->request.procStates =           tableModel->formIntVectorFromColumn(0);
    serviceInfo->request.procsBeingExecuted =   tableModel->formBoolVectorFromColumn(mission_planning_enums::executing+1);
    serviceInfo->request.procsToExecute =       tableModel->formBoolVectorFromColumn(mission_planning_enums::to_be_executed+1);
    serviceInfo->request.procsToInterrupt =     tableModel->formBoolVectorFromColumn(mission_planning_enums::interrupted+1);
    serviceInfo->request.procsToResume =        tableModel->formBoolVectorFromColumn(mission_planning_enums::resume+1);
}

void mission_planning::_implSetReadOnly(bool readOnly)
{
    ui->at_home_indicator->setReadOnly(readOnly);
    ui->avoid_count_spinbox->setReadOnly(readOnly);
    ui->avoid_lockout_indicator->setReadOnly(readOnly);
    ui->backup_count_spinbox->setReadOnly(readOnly);
    ui->collision_condition_spinbox->setReadOnly(readOnly);
    ui->confirm_collect_failed_count_spinbox->setReadOnly(readOnly);
    ui->escape_condition_indicator->setReadOnly(readOnly);
    ui->escape_lockout_indicator->setReadOnly(readOnly);
    ui->examine_count_spinbox->setReadOnly(readOnly);
    ui->execute_avoid_indicator->setReadOnly(readOnly);
    ui->give_up_roi_indicator->setReadOnly(readOnly);
    ui->homing_update_failed_indicator->setReadOnly(readOnly);
    ui->homin_update_failed_count_spinbox->setReadOnly(readOnly);
    ui->initialized_indicator->setReadOnly(readOnly);
    ui->in_deposit_pos_indicator->setReadOnly(readOnly);
    ui->in_safe_mode_indicator->setReadOnly(readOnly);
    ui->in_search_region_indicator->setReadOnly(readOnly);
    ui->mission_ended_indicator->setReadOnly(readOnly);
    ui->num_procs_spinbox->setReadOnly(readOnly);
    ui->pause_indicator->setReadOnly(readOnly);
    ui->perform_bias_removal_indicator->setReadOnly(readOnly);
    ui->perform_homing_indicator->setReadOnly(readOnly);
    ui->possessing_sample_indicator->setReadOnly(readOnly);
    ui->possession_confirmed_indicator->setReadOnly(readOnly);
    ui->possible_sample_sighted_indicator->setReadOnly(readOnly);
    ui->possibly_lost_indicator->setReadOnly(readOnly);
    ui->roi_keyframe_indicator->setReadOnly(readOnly);
    ui->roi_time_expired_indicator->setReadOnly(readOnly);
    ui->sample_collected_spinbox->setReadOnly(readOnly);
    ui->sample_in_collect_pos_indicator->setReadOnly(readOnly);
    ui->sample_sighted_indicator->setReadOnly(readOnly);
    ui->side_grab_indicator->setReadOnly(readOnly);
    ui->start_slam_indicator->setReadOnly(readOnly);
    ui->use_dead_reckoning_indicator->setReadOnly(readOnly);

    int column = 0;
    for(int row = 0; tableModel->hasIndex(row, column); row++)
    {
        for(column = 0; tableModel->hasIndex(row, column); column++)
        {
            tableModel->setData(tableModel->index(row,column, QModelIndex()), readOnly, QT_READ_ONLY_ROLE);
        }
        column = 0;
    }
}
