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

#include "exec_info_queue.h"
#include "ui_exec_info_queue_form.h"

exec_info_queue::exec_info_queue(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::exec_info_queue_form)
{
    ui->setupUi(this);

    exec_action_model *actionModel = new exec_action_model(this);

    ui->exec_table->setModel(actionModel);
    connect(this, &exec_info_queue::add_new_exec_action,
                &worker, &ros_workers::on_run_add_exec_action);
}

exec_info_queue::~exec_info_queue()
{
    delete ui;
}


void exec_info_queue::on_turn_flag_button_clicked(bool checked)
{
    compile_and_send_exec_stops();
}

void exec_info_queue::on_stop_flag_button_clicked(bool checked)
{
    compile_and_send_exec_stops();
}

void exec_info_queue::on_pause_flag_button_clicked(bool checked)
{
    compile_and_send_exec_stops();
}

void exec_info_queue::compile_and_send_exec_stops()
{
//    messages::ExecAction newAction;
//    newAction.request.pause = ui->pause_flag_button->isChecked();
//    newAction.request.pauseUnchanged = false;
//    emit add_new_exec_action(newAction);
}
