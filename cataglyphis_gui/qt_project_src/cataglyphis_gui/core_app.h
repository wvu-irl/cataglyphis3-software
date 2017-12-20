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

#ifndef CATAGLYPHIS_GUI_H
#define CATAGLYPHIS_GUI_H

#include <QMainWindow>
#include <QTabWidget>
#include <QMenuBar>
#include <QThread>
#include <boost/smart_ptr.hpp>
#include <boost/atomic.hpp>
#include <ros/ros.h>
#include "manual_control.h"
#include "init_container.h"
#include "map_viewer.h"
#include "exec_info_queue.h"
#include "mission_planning.h"

#include <QTime>
#include <QVector2D>

#include <messages/RobotPose.h>

#include <ros_workers.h>

#define NUM_MSG_CALLBACK_THREADS 2
#define CATAGLYPHIS_GUI_ID 3

namespace Ui {
class core_app_form;
}

class core_app : public QMainWindow
{
    Q_OBJECT

public:
    explicit core_app(QWidget *parent = 0, boost::shared_ptr<ros::NodeHandle> nh =
                                                    boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle()));
    ~core_app();

    Ui::core_app_form *ui;
    //boost::shared_ptr<Ui::cataglyphis_gui> ui;

public slots:
    void on_hsm_global_pose_callback(const messages::RobotPose hsmRobotPose);
    void on_update_time(double time);
private slots:

    void on_reset_distance_travelled_button_clicked();

private:

    messages::RobotPose lastRobotPose;

    boost::shared_ptr<manual_control> manualControlFormPtr;
    boost::shared_ptr<init_container> cataglyphisStartupFormPtr;
    boost::shared_ptr<map_viewer> mapViewFormPtr;
    boost::shared_ptr<exec_info_queue> execInfoFormPtr;
    boost::shared_ptr<mission_planning> missionPlanningInfoFormPtr;

    boost::shared_ptr<ros::NodeHandle> guiNhPtr;
    boost::shared_ptr<ros::AsyncSpinner> asyncSpinnerPtr;

    QThread rosWorkerThread;
    boost::shared_ptr<ros_workers> rosWorker;

};

#endif // CATAGLYPHIS_GUI_H
