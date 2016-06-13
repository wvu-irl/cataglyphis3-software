#ifndef CATAGLYPHIS_STARTUP_FORM_MAIN_H
#define CATAGLYPHIS_STARTUP_FORM_MAIN_H

#include <QWidget>
#include <QThread>

#include <boost/smart_ptr.hpp>

#include <ros/ros.h>

#include <init_step_one.h>
#include <bias_removal_form.h>
#include <generic_error_dialog_form.h>

#include <messages/NavFilterOut.h>
#include <messages/NavFilterControl.h>

#include <ros_workers.h>

namespace Ui {
class cataglyphis_startup_form_main;
}

class cataglyphis_startup_form_main : public QWidget
{
    Q_OBJECT

public:
    explicit cataglyphis_startup_form_main(QWidget *parent = 0,
                                            boost::shared_ptr<ros::NodeHandle> nh_arg =
                                                boost::shared_ptr<ros::NodeHandle>());
    ~cataglyphis_startup_form_main();

    boost::shared_ptr<Ui::cataglyphis_startup_form_main> ui;

public slots:
    void step_one_returned();
    void step_two_returned();

private slots:
    void on_start_up_button_clicked();
    void on_reboot_recovery_button_clicked();

private:
    QThread rosWorkerThread;
    boost::shared_ptr<ros_workers> rosWorker;
    boost::shared_ptr<ros::NodeHandle> nh;
    boost::shared_ptr<init_step_one> stepOneTab;
    boost::shared_ptr<bias_removal_form> stepTwoTab;
    QAtomicPointer<boost::shared_ptr<messages::NavFilterControl> > atomicNavControlSrv;
    QAtomicPointer<boost::shared_ptr<messages::NavFilterOut> > atomicNavOutMsg;

    void nav_callback(const messages::NavFilterOut::ConstPtr& callbackMsg);
};

#endif // CATAGLYPHIS_STARTUP_FORM_MAIN_H
