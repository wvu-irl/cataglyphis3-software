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
    connect(this, &shift_map::nav_service_request,
                &rosWorker, &ros_workers::on_run_nav_service);
    connect(&rosWorker, &ros_workers::map_manager_set_starting_platform_service_returned,
                this, &shift_map::on_map_manager_start_platform_set_returned);
    connect(&rosWorker, &ros_workers::nav_service_returned,
                this, &shift_map::on_nav_service_returned);

    navServiceGood = false;
    mapManagerServiceGood = false;
}

shift_map::~shift_map()
{
    delete ui;
}

void shift_map::on_submit_map_shift_button_clicked()
{
    bool doSomething = false;
    ROS_DEBUG("Shift Map:: Map shift submission");
    startingPlatformServiceRequestPtr.reset(new messages::SetStartingPlatform());
    messages::NavFilterControl navControlMsg;
    if(ui->platform_button_group->checkedId() > 0)
    {
        if(ui->set_platform_checkbox->isChecked())
        {
            startingPlatformServiceRequestPtr->request.startingPlatformNum = ui->platform_button_group->checkedId();
            navControlMsg.request.setPlatform = true;
            navControlMsg.request.platformNum = ui->platform_button_group->checkedId();
            doSomething = true;
        }

        if(ui->platform_adjustment_checkbox->isChecked())
        {
            startingPlatformServiceRequestPtr->request.deltaE = ui->delta_east_spinbox->value();
            startingPlatformServiceRequestPtr->request.deltaN = ui->delta_north_spinbox->value();
            doSomething = true;
        }

        if(doSomething)
        {
            emit set_starting_platform(*startingPlatformServiceRequestPtr);
            emit nav_service_request(navControlMsg);
        }
    }
}

void shift_map::on_map_manager_start_platform_set_returned(messages::SetStartingPlatform response, bool wasSuccessful)
{
    ROS_DEBUG("Shift Map:: starting platform service returned!");
    if(!wasSuccessful)
    {
        ROS_WARN("Shift Map:: Service was not successful!");
    }
    mapManagerServiceGood = wasSuccessful;

    if(navServiceGood && mapManagerServiceGood)
    {
        ui->map_shift_success_indicator->setChecked(wasSuccessful);
    }
}

void shift_map::on_nav_service_returned(messages::NavFilterControl response, bool wasSuccessful)
{
    ROS_DEBUG("Shift Map:: nav service returned!");
    if(!wasSuccessful)
    {
        ROS_WARN("shift map:: service was not successful!");
    }
    navServiceGood = wasSuccessful;

    if(navServiceGood && mapManagerServiceGood)
    {
        ui->map_shift_success_indicator->setChecked(wasSuccessful);
    }
}

void shift_map::on_reset_platform_adjustment_button_clicked()
{
    ROS_DEBUG("Shift Map:: reset map shift");
    startingPlatformServiceRequestPtr.reset(new messages::SetStartingPlatform());
    startingPlatformServiceRequestPtr->request.resetFineAdjustment = true;
    emit set_starting_platform(*startingPlatformServiceRequestPtr);
}
