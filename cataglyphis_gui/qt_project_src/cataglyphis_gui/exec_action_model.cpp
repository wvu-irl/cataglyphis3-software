#include "exec_action_model.h"

void exec_action_model::addExecActionToList(int actionNumber)
{
    actionList.append(new exec_action_item(lastExecInfoMsg, actionNumber));
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
