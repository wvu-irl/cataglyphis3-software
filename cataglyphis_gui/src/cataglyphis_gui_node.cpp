#include <ros/ros.h>
#include "../../robot_control/include/robot_control/cataglyphis_timer.h"
#include <ros/timer.h>

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
//    Q_INIT_RESOURCE(resources);
    ros::init(argc, argv, "GUI_Node", ros::init_options::AnonymousName);
    ROS_INFO("GUI_Node - ros::init complete");
    boost::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());
    ROS_INFO("GUI_Node - node handle created");

    QApplication qCoreApp(argc, argv);
    core_app cataglyphis_gui(0, nh);
    cataglyphis_gui.show();

    return qCoreApp.exec();
    
    //bias testClass;

    //ROS_INFO("Timer is valid? %d", testClass.timer.getInternalTimer()->isValid());
    //ROS_INFO("TIMER has pending? %d", testClass.timer.getInternalTimer()->hasPending());
//    ROS_INFO("Starting Timer");
//    testClass.timer.setPeriod(1);
//    testClass.timer.start();

//    ros::Rate loopRate(2); //set loop rate to 50Hz

//    int counter = 0;
//    while(ros::ok())
//    {
//        ROS_INFO("Counter Value %d", counter);

//        if((counter!=0)&&(counter % 20) == 0)
//        {
//            ROS_WARN("PAUSING TIMER");
//            testClass.timer.pause();
//        }
//        if((counter!=0)&&(counter % 26) == 0)
//        {
//            ROS_WARN("RESUMING Timer");
//            testClass.timer.resume();
//        }
//        //ROS_INFO("Timer is valid? %d", testClass.timer.getInternalTimer()->isValid());
//        //ROS_INFO("TIMER has pending? %d", testClass.timer.getInternalTimer()->hasPending());
//        counter++;
//        ros::spinOnce();
//        loopRate.sleep();
//    }
}


//porlar grid from starting platform

//cartesian grid relative to map NA

//local map to disply NA changes with a submit button to HSM and NAV

//each sample candidate should display location, what CV thought type, and confidence

//needto add option to push wait to front of exec queue.
//need to add commit button to map display
//


//class bias
//{
//public:
//    bias():
//        timer(&bias::callback, this)
//    {
//        j = 0;
//    }

//    void callback(const ros::TimerEvent&)
//    {
//        ROS_WARN("Timer Callback");
//        timer.start();
//    }
//    CataglyphisTimer<bias> timer;

//private:
//    int j;
//};
