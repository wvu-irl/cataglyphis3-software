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

#ifndef BIAS_REMOVAL_FORM_H
#define BIAS_REMOVAL_FORM_H

#include <QWidget>
#include <QKeyEvent>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <messages/NavFilterOut.h>
#include <messages/NavFilterControl.h>

#include <ros_workers.h>

#define INIT_STEP2_ID 2

namespace Ui {
class init_step_two_form;
}

class init_step_two : public QWidget
{
    Q_OBJECT

signals:
    void start_bias_removal();
    void start_dead_reckoning();
    void bias_removal_finished();
    void start_nav_info_subscriber();
    void stop_nav_info_subscriber();
    void stop_hsm_info_subscriber();

public:
    explicit init_step_two(QWidget *parent = 0, boost::shared_ptr<ros_workers> workerArg =
                                                            boost::shared_ptr<ros_workers>());
    ~init_step_two();

    Ui::init_step_two_form *ui;
    //boost::shared_ptr<Ui::bias_removal_form> ui;

public slots:

    void on_update_bias_removal_display(messages::NavFilterControl serviceResponse,
                                        bool wasSucessful);
    void on_nav_info_callback(const messages::NavFilterOut navInfo);

private slots:
    void on_begin_dead_reckoning_button_clicked();

    void on_perform_bias_removal_button_clicked();

private:

    boost::shared_ptr<ros::NodeHandle> nh;
    boost::shared_ptr<ros_workers> worker;
    messages::NavFilterControl navServiceResponse;

    bool previousDeadReckButtonEnabled;

protected:
    void keyPressEvent(QKeyEvent *);
    void keyReleaseEvent(QKeyEvent *);


};

#endif // BIAS_REMOVAL_FORM_H
