#ifndef ROS_WORKERS
#define ROS_WORKERS

#include <QWidget>
#include <QThread>

#include <boost/shared_ptr.hpp>
#include <QMutex>
#include <ros/ros.h>

#include <messages/NavFilterOut.h>
#include <messages/NavFilterControl.h>

class ros_workers : public QObject
{
    Q_OBJECT

signals:
    void nav_init_returned(bool sucessful);
    void bias_removal_returned(const messages::NavFilterControl navResponse);

private:
    boost::shared_ptr<ros::NodeHandle> nh;
    ros::ServiceClient navControlClient;

public:
    ros_workers(boost::shared_ptr<ros::NodeHandle> nhArg);

public slots:
    //may need to change arguments
    void run_bias_removal_service();
    void run_nav_init_service(const messages::NavFilterControl serviceRequest);

};

#endif // ROS_WORKERS

