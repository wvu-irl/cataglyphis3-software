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
