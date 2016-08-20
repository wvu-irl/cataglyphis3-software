#ifndef BIAS_REMOVAL_FORM_H
#define BIAS_REMOVAL_FORM_H

#include <QWidget>
#include <QKeyEvent>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <messages/NavFilterOut.h>
#include <messages/NavFilterControl.h>

#include <ros_workers.h>

#define INIT_STEP2_ID 2

namespace Ui {
class init_step_two_form;
}

class init_step_two : public QWidget
{
    Q_OBJECT

signals:
    void start_bias_removal();
    void start_dead_reckoning();
    void bias_removal_finished();
    void start_nav_info_subscriber();
    void stop_nav_info_subscriber();
    void stop_hsm_info_subscriber();

public:
    explicit init_step_two(QWidget *parent = 0, boost::shared_ptr<ros_workers> workerArg =
                                                            boost::shared_ptr<ros_workers>());
    ~init_step_two();

    Ui::init_step_two_form *ui;
    //boost::shared_ptr<Ui::bias_removal_form> ui;

public slots:

    void on_update_bias_removal_display(messages::NavFilterControl serviceResponse,
                                        bool wasSucessful);
    void on_nav_info_callback(const messages::NavFilterOut navInfo);

private slots:
    void on_begin_dead_reckoning_button_clicked();

    void on_perform_bias_removal_button_clicked();

private:

    boost::shared_ptr<ros::NodeHandle> nh;
    boost::shared_ptr<ros_workers> worker;
    messages::NavFilterControl navServiceResponse;

    bool previousDeadReckButtonEnabled;

protected:
    void keyPressEvent(QKeyEvent *);
    void keyReleaseEvent(QKeyEvent *);


};

#endif // BIAS_REMOVAL_FORM_H
