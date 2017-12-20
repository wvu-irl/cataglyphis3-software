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

#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H

#include <QWidget>
#include <QTimer>

#include <ros/ros.h>
#include <messages/ActuatorOut.h>
#include <messages/ExecInfo.h>

#define fully_open 1000
#define fully_closed -900
#define fully_dropped 1000
#define fully_raised -1000

namespace Ui {
class manual_control_form;
}

class manual_control : public QWidget
{
    Q_OBJECT

public slots:
    void on_manual_override(bool overrideEnabled);

public:
    explicit manual_control(QWidget *parent = 0);
    ~manual_control();

private slots:

    void on_forward_button_pressed();
    void on_forward_button_released();

    void on_backward_button_pressed();
    void on_backward_button_released();

    void on_rotate_right_button_pressed();
    void on_rotate_right_button_released();

    void on_rotate_left_button_pressed();
    void on_rotate_left_button_released();

    void on_slide_open_button_clicked();
    void on_slide_close_button_clicked();

    void on_grabber_up_button_clicked();
    void on_grabber_down_button_clicked();

private:
    Ui::manual_control_form *ui;

    bool keysEnabled;

    ros::NodeHandle nh;

    ros::Publisher actuatorPub;
    messages::ActuatorOut actuatorMsg;

    ros::Publisher execInfoPub;
    messages::ExecInfo execMsg;

};

#endif // MANUAL_CONTROL_H
