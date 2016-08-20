#ifndef CATAGLYPHIS_STARTUP_FORM_MAIN_H
#define CATAGLYPHIS_STARTUP_FORM_MAIN_H

#include <QWidget>
#include <QThread>

#include <boost/smart_ptr.hpp>

#include <ros/ros.h>

#include <init_step_one.h>
#include <init_step_two.h>
#include <shift_map.h>
#include <teleport.h>

#include <generic_error_dialog.h>

#include <messages/NavFilterOut.h>
#include <messages/NavFilterControl.h>

#include <ros_workers.h>

namespace Ui {
class init_container_form;
}

class init_container : public QWidget
{
    Q_OBJECT

public:
    explicit init_container(QWidget *parent = 0,
                                            boost::shared_ptr<ros::NodeHandle> nhArg =
                                                boost::shared_ptr<ros::NodeHandle>());
    ~init_container();

    //boost::shared_ptr<Ui::cataglyphis_startup_form_main> ui;
    Ui::init_container_form *ui;

public slots:
    void on_step_one_returned();
    void on_step_two_returned();

    void on_procedure_returned();

private slots:
    void on_start_up_button_clicked();
    void on_reboot_recovery_button_clicked();

    void on_input_tabber_currentChanged(int index);

    void on_shift_map_button_clicked();

    void on_teleport_button_clicked();

private:
    QThread rosWorkerThread;
    boost::shared_ptr<ros_workers> rosWorker;
    boost::shared_ptr<ros::NodeHandle> nh;
    boost::shared_ptr<init_step_one> northAngleTab;
    boost::shared_ptr<init_step_two> biasRemovalTab;
    boost::scoped_ptr<shift_map> shiftMapTab;
    boost::scoped_ptr<teleport> teleportTab;

    bool procedureInProgress;

    bool _implIsProcedureInProgress();
    void _implResetTabberIfNeccesary();

};

#endif // CATAGLYPHIS_STARTUP_FORM_MAIN_H
