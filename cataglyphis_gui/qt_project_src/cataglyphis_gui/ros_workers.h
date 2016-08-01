#ifndef ROS_WORKERS
#define ROS_WORKERS

#include <QWidget>
#include <QThread>

#include <boost/shared_ptr.hpp>
#include <QMutex>
#include <ros/ros.h>

#include <messages/NavFilterOut.h>
#include <messages/NavFilterControl.h>

#include <messages/HSMSetNorthAngle.h>

#define ON_SERIVCE_FAILURE_RETURN_PAUSE 3
#define NAV_INFO_MIN_PUB_TIME 1.00

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

    void nav_info_callback(const messages::NavFilterOut navInfo);

private:
    boost::shared_ptr<ros::NodeHandle> nh;
    ros::ServiceClient navControlClient;
    messages::NavFilterOut lastNavMsg;

    ros::ServiceClient hsmNAControlClient;
    messages::HSMSetNorthAngle lastHSMNAMsg;

    ros::Time navInfoTime;
    bool navInfoSubStarted;
    ros::Subscriber navInfoCallbackSub;

    void getNavInfoCallback(const messages::NavFilterOut::ConstPtr &msg);

public:
    ros_workers(boost::shared_ptr<ros::NodeHandle> nhArg);




public slots:
    //may need to change arguments
    //this slot takes a serviceName in a QString, just wrap a normal string with QString
    //ROS service messages are able to trivally cast to ros::SerializedMessage, use a normal serviceMsg.
//    void run_service(const QString serviceName,
//                              ros::SerializedMessage &request,
//                              ros::SerializedMessage &response);
    void on_run_nav_service(messages::NavFilterControl serviceRequest, const int callerID);
    void on_run_bias_removal_service();
    void on_run_start_dead_reckoning_service();
    void on_run_nav_init_service(messages::NavFilterControl serviceRequest);
    void on_run_nav_info_subscriber_start();
    void on_run_nav_info_subscriber_stop();

};

#endif // ROS_WORKERS

