#ifndef CATAGLYPHIS_GUI_H
#define CATAGLYPHIS_GUI_H

#include <QMainWindow>
#include <QTabWidget>
#include <boost/smart_ptr.hpp>
#include <boost/atomic.hpp>
#include <ros/ros.h>
#include "cataglyphis_startup_form_main.h"
#include "map_viewer.h"

#define NUM_MSG_CALLBACK_THREADS 2

namespace Ui {
class cataglyphis_gui;
}

class cataglyphis_gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit cataglyphis_gui(QWidget *parent = 0, boost::shared_ptr<ros::NodeHandle> nh =
                                                    boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle()));
    ~cataglyphis_gui();

    boost::shared_ptr<Ui::cataglyphis_gui> ui;
private slots:

private:

    boost::shared_ptr<cataglyphis_startup_form_main> cataglyphis_startup_form;
    boost::shared_ptr<map_viewer> map_view_form;

    boost::shared_ptr<ros::NodeHandle> gui_nh;
    boost::shared_ptr<ros::AsyncSpinner> async_spinner;

};

#endif // CATAGLYPHIS_GUI_H
