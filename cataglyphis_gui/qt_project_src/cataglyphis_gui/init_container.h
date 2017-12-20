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

#ifndef CATAGLYPHIS_STARTUP_FORM_MAIN_H
#define CATAGLYPHIS_STARTUP_FORM_MAIN_H

#include <QWidget>
#include <QThread>

#include <boost/smart_ptr.hpp>

#include <ros/ros.h>

#include <init_step_one.h>
#include <init_step_two.h>
#include <shift_map.h>
#include <teleport.h>

#include <generic_error_dialog.h>

#include <messages/NavFilterOut.h>
#include <messages/NavFilterControl.h>

#include <ros_workers.h>

namespace Ui {
class init_container_form;
}

class init_container : public QWidget
{
    Q_OBJECT

public:
    explicit init_container(QWidget *parent = 0);
    ~init_container();

    //boost::shared_ptr<Ui::cataglyphis_startup_form_main> ui;
    Ui::init_container_form *ui;

public slots:
    void on_step_one_returned();
    void on_step_two_returned();

    void on_procedure_returned();

private slots:
    void on_start_up_button_clicked();
    void on_reboot_recovery_button_clicked();

    void on_input_tabber_currentChanged(int index);

    void on_shift_map_button_clicked();

    void on_teleport_button_clicked();

private:
    QThread rosWorkerThread;
    boost::shared_ptr<ros_workers> rosWorker;
    boost::shared_ptr<ros::NodeHandle> nh;
    boost::shared_ptr<init_step_one> northAngleTab;
    boost::shared_ptr<init_step_two> biasRemovalTab;
    boost::scoped_ptr<shift_map> shiftMapTab;
    boost::scoped_ptr<teleport> teleportTab;

    bool procedureInProgress;

    bool _implIsProcedureInProgress();
    void _implResetTabberIfNeccesary();

};

#endif // CATAGLYPHIS_STARTUP_FORM_MAIN_H
