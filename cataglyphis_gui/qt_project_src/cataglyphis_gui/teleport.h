#ifndef TELEPORT_FORM_H
#define TELEPORT_FORM_H

#include <QWidget>
#include <ros/ros.h>
#include <ros_workers.h>
#include <messages/NavFilterControl.h>
#include <boost/scoped_ptr.hpp>

namespace Ui {
class teleport_form;
}

class teleport : public QWidget
{
    Q_OBJECT

signals:
    void navigation_teleport_robot(messages::NavFilterControl request);
    void add_wait_to_exec(float seconds);

public slots:
    void on_nav_filter_service_returned(messages::NavFilterControl response, bool wasSuccessful);

public:
    explicit teleport(QWidget *parent = 0);
    ~teleport();

private slots:
    void on_submit_teleport_button_clicked();

private:
    Ui::teleport_form *ui;

    ros_workers rosWorker;

    boost::scoped_ptr<messages::NavFilterControl> navControlMsgPtr;
};

#endif // TELEPORT_FORM_H
