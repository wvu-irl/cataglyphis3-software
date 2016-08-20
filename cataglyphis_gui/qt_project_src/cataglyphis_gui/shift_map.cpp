#include "shift_map.h"
#include "ui_shift_map_form.h"

shift_map::shift_map(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::shift_map_form)
{
    ui->setupUi(this);
    ui->platform_button_group->setId(ui->platform_one_radio, 1);
    ui->platform_button_group->setId(ui->platform_two_radio, 2);
    ui->platform_button_group->setId(ui->platform_three_radio, 3);

    connect(this, &shift_map::set_starting_platform,
                &rosWorker, &ros_workers::on_run_set_starting_platform_service);
    connect(&rosWorker, &ros_workers::map_manager_set_starting_platform_service_returned,
                this, &shift_map::on_map_manager_start_platform_set_returned);
}

shift_map::~shift_map()
{
    delete ui;
}

void shift_map::on_submit_map_shift_button_clicked()
{
    ROS_DEBUG("Shift Map:: Map shift submission");
    startingPlatformServiceRequestPtr.reset(new messages::SetStartingPlatform());
    if(ui->platform_button_group->checkedId() > 0)
    {
        startingPlatformServiceRequestPtr->request.startingPlatformNum = ui->platform_button_group->checkedId();
        if(ui->platform_adjustment_checkbox->isChecked())
        {
            startingPlatformServiceRequestPtr->request.deltaE = ui->delta_east_spinbox->value();
            startingPlatformServiceRequestPtr->request.deltaN = ui->delta_north_spinbox->value();
        }
        emit set_starting_platform(*startingPlatformServiceRequestPtr);
    }
}

void shift_map::on_map_manager_start_platform_set_returned(messages::SetStartingPlatform response, bool wasSuccessful)
{
    ROS_DEBUG("Shift Map:: starting platform service returned!");
    if(!wasSuccessful)
    {
        ROS_WARN("Shift Map:: Service was not successful!");
    }
}

void shift_map::on_reset_platform_adjustment_button_clicked()
{
    ROS_DEBUG("Shift Map:: reset map shift");
    startingPlatformServiceRequestPtr.reset(new messages::SetStartingPlatform());
    startingPlatformServiceRequestPtr->request.resetFineAdjustment = true;
    emit set_starting_platform(*startingPlatformServiceRequestPtr);
}
