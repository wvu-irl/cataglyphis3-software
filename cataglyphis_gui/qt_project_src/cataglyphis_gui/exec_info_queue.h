#ifndef EXEC_INFO_QUEUE_H
#define EXEC_INFO_QUEUE_H

#include <QWidget>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTableView>
#include <boost/scoped_ptr.hpp>
#include <messages/ExecInfo.h>

#include <exec_action_model.h>

#include <ros/ros.h>
#include <ros_workers.h>
#include <messages/ExecAction.h>

namespace Ui {
class exec_info_queue_form;
}

class exec_info_queue : public QWidget
{
    Q_OBJECT

signals:
    void add_new_exec_action(messages::ExecAction newActionToExec);

public slots:


public:
    explicit exec_info_queue(QWidget *parent = 0);
    ~exec_info_queue();

private slots:
    void on_turn_flag_button_clicked(bool checked);

    void on_stop_flag_button_clicked(bool checked);

    void on_pause_flag_button_clicked(bool checked);

private:
    Ui::exec_info_queue_form *ui;

    void compile_and_send_exec_stops();

    ros_workers worker;

};

#endif // EXEC_INFO_QUEUE_H
