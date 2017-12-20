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

#include "init_container.h"
#include "ui_init_container_form.h"
#include <QLabel>

init_container::init_container(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::init_container_form)
{
    ui->setupUi(this);

    rosWorker = boost::shared_ptr<ros_workers>(new ros_workers());
    rosWorker->moveToThread(&rosWorkerThread);

    //no need to schedule deletion of rosWorker as the boost pointer automatically does
    //connect(&rosWorkerThread, &QThread::finished, rosWorker.get(), &QObject::deleteLater);

    rosWorkerThread.start();
}

init_container::~init_container()
{
    ui->input_tabber->clear();
    rosWorkerThread.quit();
    rosWorkerThread.wait();
    std::printf("Init container destructor\r\n");
    delete ui;
}

void init_container::on_start_up_button_clicked()
{
    ROS_DEBUG("Startup Form:: Startup Button Clicked");

    _implResetTabberIfNeccesary();

    procedureInProgress = true;

    northAngleTab = boost::shared_ptr<init_step_one>(new init_step_one(ui->input_tabber, rosWorker));
    northAngleTab->hide();
    biasRemovalTab = boost::shared_ptr<init_step_two>(new init_step_two(ui->input_tabber, rosWorker));
    biasRemovalTab->hide();
    shiftMapTab.reset(new shift_map(ui->input_tabber));
    connect(northAngleTab.get(), &init_step_one::step_one_finished,
                this, &init_container::on_step_one_returned);
    connect(northAngleTab.get(), &init_step_one::procedure_finished,
                this, &init_container::on_procedure_returned);
    connect(biasRemovalTab.get(), &init_step_two::bias_removal_finished,
                this, &init_container::on_step_two_returned);
    ui->input_tabber->addTab(shiftMapTab.get(), "Shift Map");
    ui->input_tabber->addTab(northAngleTab.get(), "Nav Init");
    //ui->input_tabber->setCurrentWidget(northAngleTab.get());
    ui->input_tabber->setCurrentWidget(shiftMapTab.get());
}

void init_container::on_step_one_returned()
{
    ROS_DEBUG("Startup form:: step one finished");
    ui->input_tabber->addTab(biasRemovalTab.get(), "Bias Removal");
    ui->input_tabber->setCurrentWidget(biasRemovalTab.get());
}

void init_container::on_step_two_returned()
{
    ROS_DEBUG("Startup form:: step two finished");
    northAngleTab.reset();
    biasRemovalTab.reset();
    on_procedure_returned();
}

void init_container::on_reboot_recovery_button_clicked()
{
    ROS_DEBUG("Startup Form:: reboot recovery clicked");
}

void init_container::on_input_tabber_currentChanged(int index)
{
    std::printf("CurrentIndex:: %p\r\n", ui);
    Q_CHECK_PTR(ui->input_tabber);
    if(index >= 0 &&
            ui != 0 &&
            ui->input_tabber != 0 )
    {
        if(ui->input_tabber->currentWidget() != 0)
        {
            ui->input_tabber->resize(ui->input_tabber->currentWidget()->size());
        }
    }
}

void init_container::on_shift_map_button_clicked()
{
    ROS_DEBUG("Shift Map Button Clicked");
    on_procedure_returned();
    procedureInProgress = true;
    shiftMapTab.reset(new shift_map(ui->input_tabber));
    northAngleTab.reset(new init_step_one(ui->input_tabber, rosWorker, false));
    ui->input_tabber->addTab(shiftMapTab.get(), "Shift Map");
    ui->input_tabber->addTab(northAngleTab.get(), "North Angle");
    ui->input_tabber->setCurrentWidget(shiftMapTab.get());
}

void init_container::on_teleport_button_clicked()
{
    ROS_DEBUG("Teleport button clicked");
    on_procedure_returned();
    procedureInProgress = true;
    teleportTab.reset(new teleport(ui->input_tabber));
    ui->input_tabber->addTab(teleportTab.get(), "Teleport");
    ui->input_tabber->setCurrentWidget(teleportTab.get());
}

void init_container::on_procedure_returned()
{
    //reset pointers to tabs
    shiftMapTab.reset();
    northAngleTab.reset();
    biasRemovalTab.reset();
    teleportTab.reset();
    ui->input_tabber->clear();
    procedureInProgress = false;
}

bool init_container::_implIsProcedureInProgress()
{
    //check if procedure is in progress
    //if true, call procedure returned
    return procedureInProgress;
}

void init_container::_implResetTabberIfNeccesary()
{
    if(_implIsProcedureInProgress())
    {
        on_procedure_returned();
    }
}
