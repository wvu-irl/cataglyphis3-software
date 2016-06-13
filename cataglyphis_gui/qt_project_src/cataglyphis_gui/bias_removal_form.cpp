#include "bias_removal_form.h"
#include "ui_bias_removal_form.h"

bias_removal_form::bias_removal_form(QWidget *parent, boost::shared_ptr<ros_workers> workerArg) :
    QWidget(parent),
    ui(new Ui::bias_removal_form)
{
    ui->setupUi(this);
    worker = workerArg;

    connect(this, &bias_removal_form::start_bias_removal,
                worker.get(), &ros_workers::run_bias_removal_service);
    connect(this, &bias_removal_form::start_dead_reckoning,
                worker.get(), &ros_workers::run_start_dead_reckoning_service);
    connect(worker.get(), &ros_workers::bias_removal_returned,
                this, &bias_removal_form::update_bias_removal_display);
    connect(worker.get(), &ros_workers::dead_reckoning_service_returned,
                this, &bias_removal_form::update_bias_removal_display);
}

bias_removal_form::~bias_removal_form()
{
    //delete ui;
}

void bias_removal_form::on_begin_dead_reckoning_button_clicked()
{
    ROS_DEBUG("bias_removal_form:: bias removal finished, starting dead reckoning");
    //send service
    emit start_dead_reckoning();
    emit bias_removal_finished();
}

void bias_removal_form::on_perform_bias_removal_button_clicked()
{
    ROS_DEBUG("bias_removal_form:: starting bias removal");
    //when min and max of a progress bar are the same, it simply sits there and moves
    ui->progressBar->setMaximum(0);
    emit start_bias_removal();
}

void bias_removal_form::update_bias_removal_display(messages::NavFilterControl serviceResponse,
                                                        bool wasSucessful)
{
    ROS_DEBUG("bias_removal_form:: Updating Bias Removal Display");
    navServiceResponse = serviceResponse;
    ui->p1_offset_spinbox->setValue(navServiceResponse.response.p1Offset);
    ui->q1_offset_spinbox->setValue(navServiceResponse.response.q1Offset);
    ui->r1_offset_spinbox->setValue(navServiceResponse.response.r1Offset);

    ui->p2_offset_spinbox->setValue(navServiceResponse.response.p2Offset);
    ui->q2_offset_spinbox->setValue(navServiceResponse.response.q2Offset);
    ui->r2_offset_spinbox->setValue(navServiceResponse.response.r2Offset);

    ui->p3_offset_spinbox->setValue(navServiceResponse.response.p3Offset);
    ui->q3_offset_spinbox->setValue(navServiceResponse.response.q3Offset);
    ui->r3_offset_spinbox->setValue(navServiceResponse.response.r3Offset);
    ui->progressBar->setMaximum(100);

    if(wasSucessful)
    {
        ui->begin_dead_reckoning_button->setDisabled(false);
    }
}
