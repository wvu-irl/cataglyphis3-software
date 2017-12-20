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

#include "core_app.h"
#include "ui_core_app_form.h"

core_app::core_app(QWidget *parent, boost::shared_ptr<ros::NodeHandle> nh) :
    QMainWindow(parent),
    ui(new Ui::core_app_form)
{
    ROS_DEBUG("Cataglyphis_GUI:: Core app init");
    ui->setupUi(this);

    guiNhPtr = nh;
    ROS_DEBUG("Cataglypis_GUI:: Starting %d callback theads", NUM_MSG_CALLBACK_THREADS);
    asyncSpinnerPtr = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(NUM_MSG_CALLBACK_THREADS));
    if(asyncSpinnerPtr->canStart())
    {
        asyncSpinnerPtr->start();
    }
    else
    {
        ROS_INFO("Cataglypis_GUI:: Cannot start callback threads. Is another GUI instance running?");
    }

    if(!nh->ok())
    {
        ROS_ERROR("Node Invalid, Cannot connect to ROS Master");
    }

    //set title bar to a recongizable name
    //sharedMenuBar = boost::shared_ptr<QMenuBar>(new QMenuBar(0));
#ifdef STATIC_BUILD
    setWindowTitle(tr("Cataglyphis GUI - STATIC RELEASE"));
#elif TEST_RELEASE_BUILD
    setWindowTitle(tr("Cataglyphis GUI - TEST_RELEASE"));
#elif DEBUG_BUILD
    setWindowTitle(tr("Cataglyphis GUI - DEBUG"));
#else
    setWindowTitle(tr("Cataglyphis GUI - UNKOWN_BUILD"));
#endif


    cataglyphisStartupFormPtr.reset(new init_container(ui->guiTabber));
    mapViewFormPtr.reset(new map_viewer(ui->guiTabber, 1));
    manualControlFormPtr.reset(new manual_control(ui->guiTabber));
    execInfoFormPtr.reset(new exec_info_queue(ui->guiTabber));
    missionPlanningInfoFormPtr.reset(new mission_planning(ui->guiTabber));

    ui->guiTabber->addTab(cataglyphisStartupFormPtr.get(), "Init/Recovery");
    ui->guiTabber->addTab(mapViewFormPtr.get(), "Map");
    ui->guiTabber->addTab(manualControlFormPtr.get(), "Manual Control");
    ui->guiTabber->addTab(execInfoFormPtr.get(), "Exec Queue");
    ui->guiTabber->addTab(missionPlanningInfoFormPtr.get(), "Mission Planning");

    rosWorker = boost::shared_ptr<ros_workers>(new ros_workers());

    connect(rosWorker.get(), &ros_workers::hsm_global_pose_callback,
                this, &core_app::on_hsm_global_pose_callback);
    connect(missionPlanningInfoFormPtr.get(), &mission_planning::update_mission_timer,
                this, &core_app::on_update_time);

    rosWorker->moveToThread(&rosWorkerThread);
    rosWorker->on_run_hsm_global_pose_subscriber_start();
    rosWorkerThread.start();

}

core_app::~core_app()
{
    asyncSpinnerPtr->stop();
    rosWorkerThread.quit();
    rosWorkerThread.wait();
    delete ui;
    //ui.reset();
}


void core_app::on_hsm_global_pose_callback(const messages::RobotPose hsmRobotPose)
{
    ROS_DEBUG("Core:: HSM Callback %2.6f", QVector2D(lastRobotPose.x, lastRobotPose.y).distanceToPoint(QVector2D(hsmRobotPose.x,hsmRobotPose.y)));
    ui->distance_travelled_spinbox->setValue(ui->distance_travelled_spinbox->value()+
                                                QVector2D(lastRobotPose.x, lastRobotPose.y).distanceToPoint(QVector2D(hsmRobotPose.x,hsmRobotPose.y)));
    ui->x_spinbox->setValue(hsmRobotPose.x);
    ui->y_spinbox->setValue(hsmRobotPose.y);
    ui->heading_spinbox->setValue(hsmRobotPose.humanHeading);
    lastRobotPose = hsmRobotPose;
}

void core_app::on_update_time(double time)
{
    int hours = 0;
    int min = 0;
    int seconds = 0;

    if(time > 3600)
    {
        hours = (int)(time / 3600);
        time-=hours*3600;
    }
    if(time > 60)
    {
        min = (int)(time/60);
        time-=min*60;
    }
    if(time>0)
    {
        seconds = (int)time;
    }

    QTime t(hours, min, seconds);
    ui->mission_time->setTime(t);
}

void core_app::on_reset_distance_travelled_button_clicked()
{
    ui->distance_travelled_spinbox->setValue(0);
}
