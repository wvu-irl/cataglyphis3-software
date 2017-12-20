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

#include "teleport.h"
#include "ui_teleport_form.h"

teleport::teleport(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::teleport_form)
{
    ui->setupUi(this);

    connect(this, &teleport::navigation_teleport_robot,
                &rosWorker, &ros_workers::on_run_nav_service);
    connect(this, &teleport::add_wait_to_exec,
                &rosWorker, &ros_workers::on_add_pause_to_exec_queue);
    connect(&rosWorker, &ros_workers::nav_service_returned,
                this, &teleport::on_nav_filter_service_returned);
}

teleport::~teleport()
{
    delete ui;
}

void teleport::on_submit_teleport_button_clicked()
{
    ROS_DEBUG("Teleporting!");
    navControlMsgPtr.reset(new messages::NavFilterControl());
    navControlMsgPtr->request.newX = ui->new_x_spinbox->value();
    navControlMsgPtr->request.newY = ui->new_y_spinbox->value();
    navControlMsgPtr->request.newHeading = ui->new_heading_spinbox->value();
    navControlMsgPtr->request.setGlobalPose = true;
    emit navigation_teleport_robot(*navControlMsgPtr);
    emit add_wait_to_exec(0.05);
}

void teleport::on_nav_filter_service_returned(messages::NavFilterControl response, bool wasSuccessful)
{
    ROS_DEBUG("Teleport:: nav service returned %d", (int)wasSuccessful);
}
