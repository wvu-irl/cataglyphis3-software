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

#include "manual_control.h"
#include "ui_manual_control_form.h"

manual_control::manual_control(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::manual_control_form)
{
    ui->setupUi(this);
    keysEnabled = false;
}

manual_control::~manual_control()
{
    delete ui;
}

void manual_control::on_manual_override(bool overrideEnabled)
{
    keysEnabled = overrideEnabled;
}

void manual_control::on_forward_button_pressed()
{
    ROS_DEBUG("Manual Control:: Forward Pressed");
}
void manual_control::on_forward_button_released()
{
    ROS_DEBUG("Manual Control:: Forward Released");
}

void manual_control::on_backward_button_pressed()
{
    ROS_DEBUG("Manual Control:: Backward Pressed");
}
void manual_control::on_backward_button_released()
{
    ROS_DEBUG("Manual Control:: Backward Released");
}

void manual_control::on_rotate_right_button_pressed()
{
    ROS_DEBUG("Manual Control:: Rotate Right Pressed");
}
void manual_control::on_rotate_right_button_released()
{
    ROS_DEBUG("Manual Control:: Rotate Right Released");
}

void manual_control::on_rotate_left_button_pressed()
{
    ROS_DEBUG("Manual Control:: Rotate Left Pressed");
}
void manual_control::on_rotate_left_button_released()
{
    ROS_DEBUG("Manual Control:: Rotate Left Released");
}

void manual_control::on_slide_open_button_clicked()
{

}

void manual_control::on_grabber_up_button_clicked()
{

}

void manual_control::on_slide_close_button_clicked()
{

}

void manual_control::on_grabber_down_button_clicked()
{

}
