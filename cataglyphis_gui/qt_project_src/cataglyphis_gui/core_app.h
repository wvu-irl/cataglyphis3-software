#ifndef CATAGLYPHIS_GUI_H
#define CATAGLYPHIS_GUI_H

#include <QMainWindow>
#include <QTabWidget>
#include <QMenuBar>
#include <boost/smart_ptr.hpp>
#include <boost/atomic.hpp>
#include <ros/ros.h>
#include "manual_control.h"
#include "init_container.h"
#include "map_viewer.h"

#define NUM_MSG_CALLBACK_THREADS 2
#define CATAGLYPHIS_GUI_ID 3

namespace Ui {
class core_app_form;
}

class core_app : public QMainWindow
{
    Q_OBJECT

public:
    explicit core_app(QWidget *parent = 0, boost::shared_ptr<ros::NodeHandle> nh =
                                                    boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle()));
    ~core_app();

    Ui::core_app_form *ui;
    //boost::shared_ptr<Ui::cataglyphis_gui> ui;
private slots:

private:

    boost::shared_ptr<manual_control> manualControlFormPtr;
    boost::shared_ptr<init_container> cataglyphisStartupFormPtr;
    boost::shared_ptr<map_viewer> mapViewFormPtr;

    boost::shared_ptr<ros::NodeHandle> guiNhPtr;
    boost::shared_ptr<ros::AsyncSpinner> asyncSpinnerPtr;

};

#endif // CATAGLYPHIS_GUI_H
