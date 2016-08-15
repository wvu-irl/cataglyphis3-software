#include <ros/ros.h>

#ifdef STATIC
    #include <QtPlugin>
#endif


#include <QApplication>
#include "core_app.h"

int main(int argc, char **argv)
{

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }
    //Q_INIT_RESOURCE(resources);
    ros::init(argc, argv, "GUI_Node", ros::init_options::AnonymousName);
    ROS_INFO("GUI_Node - ros::init complete");
    boost::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());
    ROS_INFO("GUI_Node - node handle created");

    QApplication qCoreApp(argc, argv);
    core_app cataglyphis_gui(0, nh);
    cataglyphis_gui.show();

    return qCoreApp.exec();
    
    ros::Rate loopRate(50); //set loop rate to 50Hz

    ros::spinOnce();
    loopRate.sleep();

    
}


//porlar grid from starting platform

//cartesian grid relative to map NA


//local map to disply NA changes with a submit button to HSM and NAV

//each sample candidate should display location, what CV thought type, and confidence
