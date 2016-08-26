#include "exec_info_queue.h"
#include "ui_exec_info_queue_form.h"

exec_info_queue::exec_info_queue(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::exec_info_queue_form)
{
    ui->setupUi(this);

    exec_action_model *actionModel = new exec_action_model(this);

    ui->exec_table->setModel(actionModel);
    connect(this, &exec_info_queue::add_new_exec_action,
                &worker, &ros_workers::on_run_add_exec_action);
}

exec_info_queue::~exec_info_queue()
{
    delete ui;
}


void exec_info_queue::on_turn_flag_button_clicked(bool checked)
{
    compile_and_send_exec_stops();
}

void exec_info_queue::on_stop_flag_button_clicked(bool checked)
{
    compile_and_send_exec_stops();
}

void exec_info_queue::on_pause_flag_button_clicked(bool checked)
{
    compile_and_send_exec_stops();
}

void exec_info_queue::compile_and_send_exec_stops()
{
//    messages::ExecAction newAction;
//    newAction.request.pause = ui->pause_flag_button->isChecked();
//    newAction.request.pauseUnchanged = false;
//    emit add_new_exec_action(newAction);
}
