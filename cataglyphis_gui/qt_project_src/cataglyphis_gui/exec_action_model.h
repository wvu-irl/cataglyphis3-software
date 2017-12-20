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

#ifndef EXEC_ACTION_MODEL_H
#define EXEC_ACTION_MODEL_H

#include <QObject>
#include <QStandardItemModel>
#include <QThread>
#include <QList>
#include <QStringList>

#include <exec_action_item.h>

#include <ros_workers.h>

#include <messages/ExecInfo.h>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

class exec_action_model : public QStandardItemModel
{
    Q_OBJECT

signals:
    void start_exec_info_callback();
    void stop_exec_info_callback();

public slots:
    void on_exec_info_callback(const messages::ExecInfo msg)
    {
        if(msg.actionDequeSize != lastExecInfoMsg.actionDequeSize)
        {
            lastExecInfoMsg = msg;
            clearTable();
            addAllActions();
        }
    }

public:
    exec_action_model(QObject *parent = 0): QStandardItemModel(parent)
    {
        setupTable();
        rosWorker.reset(new ros_workers());
        rosWorker->moveToThread(&rosWorkerThread);
        rosWorkerThread.start();

        connect(this, &exec_action_model::start_exec_info_callback,
                    rosWorker.get(), &ros_workers::on_run_exec_info_subscriber_start);
        connect(this, &exec_action_model::stop_exec_info_callback,
                    rosWorker.get(), &ros_workers::on_run_exec_info_subscriber_stop);
        connect(rosWorker.get(), &ros_workers::exec_info_callback,
                    this, &exec_action_model::on_exec_info_callback);
        emit start_exec_info_callback();
    }
    ~exec_action_model()
    {
        rosWorkerThread.quit();
        rosWorkerThread.wait();
    }

    void addExecActionToList(int actionNumber);

    void addAllActions()
    {
        for(unsigned int i = 0; i < lastExecInfoMsg.actionDequeSize; i++)
        {
            appendColumn(createActionColumn(lastExecInfoMsg, (int)i));
        }
        ROS_DEBUG("Added all actions, rows %d, columns %d", rowCount(), columnCount());
        ROS_DEBUG("List %d", actionList.size());
    }

    QList<QStandardItem*> createActionColumn(const messages::ExecInfo &currentInfo, int actionNumber);

    void setupTable()
    {
        setHorizontalHeaderLabels(horizontalLabels);
        setVerticalHeaderLabels(verticalLabels);
    }
    void clearTable()
    {
        this->clear();
        actionList.clear();
    };
    void refreshTable()
    {
        clearTable();
        addAllActions();
    };



private:
    QThread rosWorkerThread;
    boost::scoped_ptr<ros_workers> rosWorker;

    messages::ExecInfo lastExecInfoMsg;

    QList<exec_action_item*> actionList;
    QStringList horizontalLabels;
    QStringList verticalLabels;
};

#endif // EXEC_ACTION_MODEL_H
