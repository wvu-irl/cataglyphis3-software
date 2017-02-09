#include "teleport.h"
#include "ui_teleport_form.h"

teleport::teleport(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::teleport_form)
{
    ui->setupUi(this);

    connect(this, &teleport::navigation_teleport_robot,
                &rosWorker, &ros_workers::on_run_nav_service);
    connect(this, &teleport::add_wait_to_exec,
                &rosWorker, &ros_workers::on_add_pause_to_exec_queue);
    connect(&rosWorker, &ros_workers::nav_service_returned,
                this, &teleport::on_nav_filter_service_returned);
}

teleport::~teleport()
{
    delete ui;
}

void teleport::on_submit_teleport_button_clicked()
{
    ROS_DEBUG("Teleporting!");
    navControlMsgPtr.reset(new messages::NavFilterControl());
    navControlMsgPtr->request.newX = ui->new_x_spinbox->value();
    navControlMsgPtr->request.newY = ui->new_y_spinbox->value();
    navControlMsgPtr->request.newHeading = ui->new_heading_spinbox->value();
    navControlMsgPtr->request.setGlobalPose = true;
    emit navigation_teleport_robot(*navControlMsgPtr);
    emit add_wait_to_exec(0.05);
}

void teleport::on_nav_filter_service_returned(messages::NavFilterControl response, bool wasSuccessful)
{
    ROS_DEBUG("Teleport:: nav service returned %d", (int)wasSuccessful);
}

