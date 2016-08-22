#include "exec_info_queue.h"
#include "ui_exec_info_queue_form.h"

exec_info_queue::exec_info_queue(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::exec_info_queue_form)
{
    ui->setupUi(this);
}

exec_info_queue::~exec_info_queue()
{
    delete ui;
}
