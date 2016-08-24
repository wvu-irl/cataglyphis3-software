#ifndef EXEC_ACTION_ITEM_H
#define EXEC_ACTION_ITEM_H

#include <ros/ros.h>

#include <QObject>
#include <QStandardItem>
#include <QTableWidgetItem>
#include <QList>

#include <messages/ExecInfo.h>
#include <exec_action_enums.h>

#define NUM_OF_EXEC_MSG_FIELDS 17

class exec_action_item : public QStandardItem
{
public:
    exec_action_item(/*const messages::ExecInfo &currentInfo, int actionNumber*/)
    {


        appendRows(dataItems);
    }

    QList<QStandardItem*> dataItems;
};

#endif // EXEC_ACTION_ITEM_H
