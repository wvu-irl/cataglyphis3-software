#ifndef EXEC_INFO_QUEUE_H
#define EXEC_INFO_QUEUE_H

#include <QWidget>

namespace Ui {
class exec_info_queue_form;
}

class exec_info_queue : public QWidget
{
    Q_OBJECT

public:
    explicit exec_info_queue(QWidget *parent = 0);
    ~exec_info_queue();

private:
    Ui::exec_info_queue_form *ui;
};

#endif // EXEC_INFO_QUEUE_H
