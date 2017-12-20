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

#ifndef ROI_DIALOG_H
#define ROI_DIALOG_H

#include <QDialog>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTableView>
#include <qledindicator.h>
#include <boost/scoped_ptr.hpp>

#include <robot_control/ROI.h>
#include <robot_control/ModifyROI.h>

#include <generic_ack_dialog.h>

#include <ros_workers.h>

namespace Ui {
class ROI_dialog;
}

class ROI_dialog : public QDialog
{
    Q_OBJECT

signals:
    void update_roi_ellipse(robot_control::ROI roiData, bool modified);
    void modify_roi_request(robot_control::ModifyROI service);
    void add_wait_to_exec(float seconds);

public slots:
    void accept();
    void reject();
    void open();
    void on_confirm_changes();
    void on_discard_changes();

public:
    explicit ROI_dialog(QWidget *parent = 0);
    explicit ROI_dialog(robot_control::ROI data, int roiNum,
                            boost::shared_ptr<ros_workers> workerArg = boost::shared_ptr<ros_workers>());
    ~ROI_dialog();

    bool isModified(){return modified;}
    robot_control::ROI internalROI();

private slots:
    void on_edit_roi_button_clicked(bool checked);

    void on_reset_roi_button_clicked();

private:
    Ui::ROI_dialog *ui;
    boost::shared_ptr<ros_workers> worker;

    robot_control::ROI * _implCurrentROISet();
    void _implMsgToUi(robot_control::ROI *data);
    void _implUiToMsg(robot_control::ROI *data);
    void _implSetButtonReadOnly(bool readOnly);

    bool modified;
    int roiNumber;
    robot_control::ROI ROIData;
    robot_control::ROI stagedROIData;
};

#endif // ROI_DIALOG_H
