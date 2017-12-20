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

#include "exec_action_model.h"

void exec_action_model::addExecActionToList(int actionNumber)
{
    actionList.append(new exec_action_item(/*lastExecInfoMsg, actionNumber*/));
}

QList<QStandardItem*> exec_action_model::createActionColumn(const messages::ExecInfo &currentInfo, int actionNumber)
{
    QList<QStandardItem*> dataItems;
    QStandardItem *newItem = new QStandardItem();
    newItem->setText(QString(exec_action_enums::actionTypeToString.find(static_cast<ACTION_TYPE_T>(currentInfo.actionDeque[actionNumber]))->second.c_str())); //std map between int and string
    dataItems.append(newItem);
    ROS_DEBUG("ActionItem:: Name is %s", exec_action_enums::actionTypeToString.find(static_cast<ACTION_TYPE_T>(currentInfo.actionDeque[actionNumber]))->second.c_str());

    newItem = new QStandardItem(QString::number(currentInfo.actionFloat1[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem(QString::number(currentInfo.actionFloat2[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem(QString::number(currentInfo.actionFloat3[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem(QString::number(currentInfo.actionFloat4[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem(QString::number(currentInfo.actionFloat5[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem();
    newItem->setText(QString::number(currentInfo.actionInt1[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem();
    newItem->setText(QString::number(currentInfo.actionBool1[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem();
    newItem->setText(QString::number(currentInfo.actionBool2[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem();
    newItem->setText(QString::number(currentInfo.actionBool3[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem();
    newItem->setText(QString::number(currentInfo.actionBool4[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem();
    newItem->setText(QString::number(currentInfo.actionBool5[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem();
    newItem->setText(QString::number(currentInfo.actionBool6[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem();
    newItem->setText(QString::number(currentInfo.actionBool7[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem();
    newItem->setText(QString::number(currentInfo.actionBool8[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem();
    newItem->setText(QString::number(currentInfo.actionProcType[actionNumber]));
    dataItems.append(newItem);

    newItem = new QStandardItem();
    newItem->setText(QString::number(currentInfo.actionSerialNum[actionNumber]));
    dataItems.append(newItem);

    return dataItems;
}
