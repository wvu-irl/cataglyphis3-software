#ifndef EXEC_INFO_QUEUE_H
#define EXEC_INFO_QUEUE_H

#include <QWidget>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTableView>
#include <boost/scoped_ptr.hpp>
#include <messages/ExecInfo.h>

#include <exec_action_model.h>

namespace Ui {
class exec_info_queue_form;
}

class exec_info_queue : public QWidget
{
    Q_OBJECT

public slots:


public:
    explicit exec_info_queue(QWidget *parent = 0);
    ~exec_info_queue();

private:
    Ui::exec_info_queue_form *ui;

};

#endif // EXEC_INFO_QUEUE_H
