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

#include "init_step_two.h"
#include "ui_init_step_two_form.h"

init_step_two::init_step_two(QWidget *parent, boost::shared_ptr<ros_workers> workerArg) :
    QWidget(parent),
    ui(new Ui::init_step_two_form)
{
    ui->setupUi(this);
    worker = workerArg;
    previousDeadReckButtonEnabled = false;

    connect(this, &init_step_two::start_bias_removal,
                worker.get(), &ros_workers::on_run_bias_removal_service);
    connect(this, &init_step_two::start_dead_reckoning,
                worker.get(), &ros_workers::on_run_start_dead_reckoning_service);

    connect(this, &init_step_two::start_nav_info_subscriber,
                worker.get(), &ros_workers::on_run_nav_info_subscriber_start);
    connect(this, &init_step_two::stop_nav_info_subscriber,
                worker.get(), &ros_workers::on_run_nav_info_subscriber_stop);

    connect(this, &init_step_two::stop_hsm_info_subscriber,
                worker.get(), &ros_workers::on_run_hsm_global_pose_subscriber_stop);

    connect(worker.get(), &ros_workers::bias_removal_returned,
                this, &init_step_two::on_update_bias_removal_display);
    connect(worker.get(), &ros_workers::dead_reckoning_service_returned,
                this, &init_step_two::on_update_bias_removal_display);

    connect(worker.get(), &ros_workers::nav_info_callback,
                this, &init_step_two::on_nav_info_callback);

    emit start_nav_info_subscriber();
}

init_step_two::~init_step_two()
{
    delete ui;
}

void init_step_two::on_begin_dead_reckoning_button_clicked()
{
    ROS_DEBUG("bias_removal_form:: bias removal finished, starting dead reckoning");
    //send service
    emit start_dead_reckoning();
    emit bias_removal_finished();
//    emit stop_nav_info_subscriber();
//    emit stop_hsm_info_subscriber();
}

void init_step_two::on_perform_bias_removal_button_clicked()
{
    ROS_DEBUG("bias_removal_form:: starting bias removal");
    //when min and max of a progress bar are the same, it simply sits there and moves
    ui->progressBar->setMaximum(0);
    emit start_bias_removal();
}

void init_step_two::on_update_bias_removal_display(messages::NavFilterControl serviceResponse,
                                                        bool wasSucessful)
{
    ROS_DEBUG("bias_removal_form:: Updating Bias Removal Display");
    navServiceResponse = serviceResponse;
    ui->p1_offset_spinbox->setValue(navServiceResponse.response.p1Offset);
    ui->q1_offset_spinbox->setValue(navServiceResponse.response.q1Offset);
    ui->r1_offset_spinbox->setValue(navServiceResponse.response.r1Offset);

    ui->p2_offset_spinbox->setValue(navServiceResponse.response.p2Offset);
    ui->q2_offset_spinbox->setValue(navServiceResponse.response.q2Offset);
    ui->r2_offset_spinbox->setValue(navServiceResponse.response.r2Offset);

    ui->p3_offset_spinbox->setValue(navServiceResponse.response.p3Offset);
    ui->q3_offset_spinbox->setValue(navServiceResponse.response.q3Offset);
    ui->r3_offset_spinbox->setValue(navServiceResponse.response.r3Offset);
    ui->progressBar->setMaximum(100);

    if(wasSucessful)
    {
        ui->begin_dead_reckoning_button->setDisabled(false);
    }
}

void init_step_two::on_nav_info_callback(const messages::NavFilterOut navInfo)
{
    ROS_DEBUG("bias_removal_form:: nav_info_callback");
    ui->p1_offset_spinbox->setValue(navInfo.p1_offset);
    ui->q1_offset_spinbox->setValue(navInfo.q1_offset);
    ui->r1_offset_spinbox->setValue(navInfo.r1_offset);

    ui->p2_offset_spinbox->setValue(navInfo.p2_offset);
    ui->q2_offset_spinbox->setValue(navInfo.q2_offset);
    ui->r2_offset_spinbox->setValue(navInfo.r2_offset);

    ui->p3_offset_spinbox->setValue(navInfo.p3_offset);
    ui->q3_offset_spinbox->setValue(navInfo.q3_offset);
    ui->r3_offset_spinbox->setValue(navInfo.r3_offset);
}

void init_step_two::keyPressEvent(QKeyEvent *event)
{
    ROS_DEBUG("bias_removal_form:: Key Pressed");
    if((event->modifiers() & Qt::ControlModifier) == Qt::ControlModifier)
    {
        ROS_DEBUG("bias_removal_form:: Control pressed");
        previousDeadReckButtonEnabled = ui->begin_dead_reckoning_button->isEnabled();
        ui->begin_dead_reckoning_button->setEnabled(true);
    }

}

void init_step_two::keyReleaseEvent(QKeyEvent *event)
{
    if((event->modifiers() & Qt::ControlModifier) != Qt::ControlModifier)
    {
        ROS_DEBUG("bias_removal_form:: Control released");
        ui->begin_dead_reckoning_button->setEnabled(previousDeadReckButtonEnabled);
    }
}
