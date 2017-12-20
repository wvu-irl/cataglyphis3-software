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

#ifndef INIT_STEP_ONE_H
#define INIT_STEP_ONE_H

#include <QWidget>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <ros_workers.h>

#include <messages/NavFilterOut.h>
#include <messages/RobotPose.h>

#define INIT_STEP1_ID 1

namespace Ui {
class init_step_one_form;
}

class init_step_one : public QWidget
{
    Q_OBJECT

signals:
    void init_nav_filter(messages::NavFilterControl serviceRequest);
    void step_one_finished();
    void start_nav_info_subscriber();
    void stop_nav_info_subscriber();
    void start_hsm_pose_subscriber();
    void stop_hsm_pose_subscriber();
    void procedure_finished();

public:
    explicit init_step_one(QWidget *parent = 0, boost::shared_ptr<ros_workers> workerArg =
                                                    boost::shared_ptr<ros_workers>(),
                                                bool startup = true);
    ~init_step_one();

    Ui::init_step_one_form *ui;
    //boost::shared_ptr<Ui::init_step_one> ui;

public slots:
    void on_nav_init_return(const messages::NavFilterControl navResponse,
                                bool sucessful);

    void on_nav_info_callback(const messages::NavFilterOut navInfo);
    void on_hsm_pose_callback(const messages::RobotPose robotPose);

private slots:

    void on_skip_init_button_clicked();

    void on_continue_button_clicked();

private:

    boost::shared_ptr<ros_workers> worker;

    bool isThisStartup;
};

#endif // INIT_STEP_ONE_H
