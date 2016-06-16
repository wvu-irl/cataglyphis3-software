#ifndef ROS_WORKERS
#define ROS_WORKERS

#include <QWidget>
#include <QThread>

#include <boost/shared_ptr.hpp>
#include <QMutex>
#include <ros/ros.h>

#include <messages/NavFilterOut.h>
#include <messages/NavFilterControl.h>

#define ON_SERIVCE_FAILURE_RETURN_PAUSE 3

class ros_workers : public QObject
{
    Q_OBJECT

signals:
    void nav_service_returned(const messages::NavFilterControl navResponse,
                                  bool wasSucessful,
                                    const int callerID);

    void nav_init_returned(const messages::NavFilterControl navResponse,
                                  bool wasSucessful);

    void bias_removal_returned(const messages::NavFilterControl navResponse,
                                           bool wasSucessful);

    void dead_reckoning_service_returned(const messages::NavFilterControl navResponse,
                                            bool wasSucessful);

private:
    boost::shared_ptr<ros::NodeHandle> nh;
    ros::ServiceClient navControlClient;

public:
    ros_workers(boost::shared_ptr<ros::NodeHandle> nhArg);

public slots:
    //may need to change arguments
    //this slot takes a serviceName in a QString, just wrap a normal string with QString
    //ROS service messages are able to trivally cast to ros::SerializedMessage, use a normal serviceMsg.
//    void run_service(const QString serviceName,
//                              ros::SerializedMessage &request,
//                              ros::SerializedMessage &response);
    void run_nav_service(messages::NavFilterControl serviceRequest, const int callerID);
    void run_bias_removal_service();
    void run_start_dead_reckoning_service();
    void run_nav_init_service(messages::NavFilterControl serviceRequest);

};

#endif // ROS_WORKERS

