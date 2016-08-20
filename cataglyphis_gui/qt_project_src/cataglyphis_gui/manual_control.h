#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H

#include <QWidget>
#include <QTimer>

#include <ros/ros.h>
#include <messages/ActuatorOut.h>
#include <messages/ExecInfo.h>

#define fully_open 1000
#define fully_closed -900
#define fully_dropped 1000
#define fully_raised -1000

namespace Ui {
class manual_control_form;
}

class manual_control : public QWidget
{
    Q_OBJECT

public slots:
    void on_manual_override(bool overrideEnabled);

public:
    explicit manual_control(QWidget *parent = 0);
    ~manual_control();

private slots:

    void on_forward_button_pressed();
    void on_forward_button_released();

    void on_backward_button_pressed();
    void on_backward_button_released();

    void on_rotate_right_button_pressed();
    void on_rotate_right_button_released();

    void on_rotate_left_button_pressed();
    void on_rotate_left_button_released();

    void on_slide_open_button_clicked();
    void on_slide_close_button_clicked();

    void on_grabber_up_button_clicked();
    void on_grabber_down_button_clicked();

private:
    Ui::manual_control_form *ui;

    bool keysEnabled;

    ros::NodeHandle nh;

    ros::Publisher actuatorPub;
    messages::ActuatorOut actuatorMsg;

    ros::Publisher execInfoPub;
    messages::ExecInfo execMsg;

};

#endif // MANUAL_CONTROL_H
