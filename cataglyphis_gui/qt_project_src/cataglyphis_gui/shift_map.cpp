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

#include "shift_map.h"
#include "ui_shift_map_form.h"

shift_map::shift_map(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::shift_map_form)
{
    ui->setupUi(this);
    ui->platform_button_group->setId(ui->platform_one_radio, 1);
    ui->platform_button_group->setId(ui->platform_two_radio, 2);
    ui->platform_button_group->setId(ui->platform_three_radio, 3);

    connect(this, &shift_map::set_starting_platform,
                &rosWorker, &ros_workers::on_run_set_starting_platform_service);
    connect(this, &shift_map::nav_service_request,
                &rosWorker, &ros_workers::on_run_nav_service);
    connect(&rosWorker, &ros_workers::map_manager_set_starting_platform_service_returned,
                this, &shift_map::on_map_manager_start_platform_set_returned);
    connect(&rosWorker, &ros_workers::nav_service_returned,
                this, &shift_map::on_nav_service_returned);

    navServiceGood = false;
    mapManagerServiceGood = false;
}

shift_map::~shift_map()
{
    delete ui;
}

void shift_map::on_submit_map_shift_button_clicked()
{
    bool doSomething = false;
    ROS_DEBUG("Shift Map:: Map shift submission");
    startingPlatformServiceRequestPtr.reset(new messages::SetStartingPlatform());
    messages::NavFilterControl navControlMsg;
    if(ui->platform_button_group->checkedId() > 0)
    {
        if(ui->set_platform_checkbox->isChecked())
        {
            startingPlatformServiceRequestPtr->request.responseOnly = false;
            startingPlatformServiceRequestPtr->request.startingPlatformNum = ui->platform_button_group->checkedId();
            navControlMsg.request.setPlatform = true;
            navControlMsg.request.platformNum = ui->platform_button_group->checkedId();
            doSomething = true;
        }

        if(ui->platform_adjustment_checkbox->isChecked())
        {
            startingPlatformServiceRequestPtr->request.deltaE = ui->delta_east_spinbox->value();
            startingPlatformServiceRequestPtr->request.deltaN = ui->delta_north_spinbox->value();
            doSomething = true;
        }

        if(doSomething)
        {
            emit set_starting_platform(*startingPlatformServiceRequestPtr);
            emit nav_service_request(navControlMsg);
        }
    }
}

void shift_map::on_map_manager_start_platform_set_returned(messages::SetStartingPlatform response, bool wasSuccessful)
{
    ROS_DEBUG("Shift Map:: starting platform service returned!");
    if(!wasSuccessful)
    {
        ROS_WARN("Shift Map:: Service was not successful!");
    }
    mapManagerServiceGood = wasSuccessful;

    if(navServiceGood && mapManagerServiceGood)
    {
        ui->map_shift_success_indicator->setChecked(wasSuccessful);
    }
}

void shift_map::on_nav_service_returned(messages::NavFilterControl response, bool wasSuccessful)
{
    ROS_DEBUG("Shift Map:: nav service returned!");
    if(!wasSuccessful)
    {
        ROS_WARN("shift map:: service was not successful!");
    }
    navServiceGood = wasSuccessful;

    if(navServiceGood && mapManagerServiceGood)
    {
        ui->map_shift_success_indicator->setChecked(wasSuccessful);
    }
}

void shift_map::on_reset_platform_adjustment_button_clicked()
{
    ROS_DEBUG("Shift Map:: reset map shift");
    startingPlatformServiceRequestPtr.reset(new messages::SetStartingPlatform());
    startingPlatformServiceRequestPtr->request.resetFineAdjustment = true;
    emit set_starting_platform(*startingPlatformServiceRequestPtr);
}
