#ifndef INIT_STEP_ONE_H
#define INIT_STEP_ONE_H

#include <QWidget>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <ros_workers.h>

#include <messages/NavFilterOut.h>
#include <messages/RobotPose.h>

#define INIT_STEP1_ID 1

namespace Ui {
class init_step_one_form;
}

class init_step_one : public QWidget
{
    Q_OBJECT

signals:
    void init_nav_filter(messages::NavFilterControl serviceRequest);
    void step_one_finished();
    void start_nav_info_subscriber();
    void stop_nav_info_subscriber();
    void start_hsm_pose_subscriber();
    void stop_hsm_pose_subscriber();
    void procedure_finished();

public:
    explicit init_step_one(QWidget *parent = 0, boost::shared_ptr<ros_workers> workerArg =
                                                    boost::shared_ptr<ros_workers>(),
                                                bool startup = true);
    ~init_step_one();

    Ui::init_step_one_form *ui;
    //boost::shared_ptr<Ui::init_step_one> ui;

public slots:
    void on_nav_init_return(const messages::NavFilterControl navResponse,
                                bool sucessful);

    void on_nav_info_callback(const messages::NavFilterOut navInfo);
    void on_hsm_pose_callback(const messages::RobotPose robotPose);

private slots:

    void on_skip_init_button_clicked();

    void on_continue_button_clicked();

private:

    boost::shared_ptr<ros_workers> worker;

    bool isThisStartup;
};

#endif // INIT_STEP_ONE_H
