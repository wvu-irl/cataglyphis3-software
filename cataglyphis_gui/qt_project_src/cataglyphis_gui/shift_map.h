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

#ifndef SHIFT_MAP_H
#define SHIFT_MAP_H

#include <QWidget>
#include <boost/scoped_ptr.hpp>
#include <ros_workers.h>
#include <messages/SetStartingPlatform.h>
#include <messages/NavFilterControl.h>
#include <qledindicator.h>

namespace Ui {
class shift_map_form;
}

class shift_map : public QWidget
{
    Q_OBJECT

signals:
    void set_starting_platform(messages::SetStartingPlatform);
    void nav_service_request(messages::NavFilterControl);

public slots:
    void on_map_manager_start_platform_set_returned(messages::SetStartingPlatform response, bool wasSuccessful);
    void on_nav_service_returned(messages::NavFilterControl response, bool wasSuccessful);

public:
    explicit shift_map(QWidget *parent = 0);
    ~shift_map();

private slots:
    void on_submit_map_shift_button_clicked();

    void on_reset_platform_adjustment_button_clicked();

private:
    Ui::shift_map_form *ui;

    ros_workers rosWorker;

    boost::scoped_ptr<messages::SetStartingPlatform> startingPlatformServiceRequestPtr;

    bool navServiceGood;
    bool mapManagerServiceGood;
};

#endif // SHIFT_MAP_H
