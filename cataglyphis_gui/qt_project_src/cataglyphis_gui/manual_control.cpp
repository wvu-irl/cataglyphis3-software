#include "manual_control.h"
#include "ui_manual_control_form.h"

manual_control::manual_control(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::manual_control_form)
{
    ui->setupUi(this);
    keysEnabled = false;
}

manual_control::~manual_control()
{
    delete ui;
}

void manual_control::on_manual_override(bool overrideEnabled)
{
    keysEnabled = overrideEnabled;
}

void manual_control::on_forward_button_pressed()
{
    ROS_DEBUG("Manual Control:: Forward Pressed");
}
void manual_control::on_forward_button_released()
{
    ROS_DEBUG("Manual Control:: Forward Released");
}

void manual_control::on_backward_button_pressed()
{
    ROS_DEBUG("Manual Control:: Backward Pressed");
}
void manual_control::on_backward_button_released()
{
    ROS_DEBUG("Manual Control:: Backward Released");
}

void manual_control::on_rotate_right_button_pressed()
{
    ROS_DEBUG("Manual Control:: Rotate Right Pressed");
}
void manual_control::on_rotate_right_button_released()
{
    ROS_DEBUG("Manual Control:: Rotate Right Released");
}

void manual_control::on_rotate_left_button_pressed()
{
    ROS_DEBUG("Manual Control:: Rotate Left Pressed");
}
void manual_control::on_rotate_left_button_released()
{
    ROS_DEBUG("Manual Control:: Rotate Left Released");
}

void manual_control::on_slide_open_button_clicked()
{

}

void manual_control::on_grabber_up_button_clicked()
{

}

void manual_control::on_slide_close_button_clicked()
{

}

void manual_control::on_grabber_down_button_clicked()
{

}
