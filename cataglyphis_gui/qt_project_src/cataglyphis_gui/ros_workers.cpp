#include <ros_workers.h>

ros_workers::ros_workers(boost::shared_ptr<ros::NodeHandle> nhArg)
{
    nh = nhArg;
    navControlClient = nh->serviceClient<messages::NavFilterControl>("/nav/nav_filter_control");
    //workerMutex = boost::shared_ptr<QMutex>(new QMutex);
}

void ros_workers::run_nav_init_service(const messages::NavFilterControl serviceRequest)
{
    bool sucessful = false;
    messages::NavFilterControl navControl = serviceRequest;

    if(navControlClient.exists())
    {
        ROS_DEBUG("ros_workers::nav_init_service:: Nav Service Exists!");
        navControl.request.runBiasRemoval = false;
        if(navControlClient.call(navControl.request, navControl.response))
        {
            ROS_DEBUG("ros_workers::nav_init_service:: Nav Service Call Sucess");
            sucessful = true;
        }
        else
        {
            ROS_WARN("ros_workers::nav_init_service:: Nav Service Call Failure!");
        }
    }
    else
    {
        ROS_WARN("ros_workers::nav_init_service:: Nav Service Does not Exist!");
        ros::Duration pause(3);
        ROS_DEBUG("ros_worker:: sleeping %d", pause.sleep());
    }
    emit nav_init_returned(sucessful);
}

void ros_workers::run_bias_removal_service()
{
    ROS_DEBUG("ros_workers:: Bias RemovalService Handler");
    messages::NavFilterControl navControl;

    if(navControlClient.exists())
    {
        ROS_DEBUG("ros_workers::bias_removal_service:: Nav Service Exists!");
        navControl.request.runBiasRemoval = true;
        if(navControlClient.call(navControl.request, navControl.response))
        {
            ROS_DEBUG("ros_workers::bias_removal_service:: Nav Service Call Sucess");
        }
        else
        {
            ROS_WARN("ros_workers::bias_removal_service:: Nav Service Call Failure!");
        }
    }
    else
    {
        ROS_WARN("ros_workers::bias_removal_service:: Nav Service Does not Exist!");
        ros::Duration pause(3);
        ROS_DEBUG("ros_worker:: sleeping %d", pause.sleep());
        navControl.response.p1Offset = 5.234;
        navControl.response.q1Offset = 5.234;
        navControl.response.r1Offset = 5.234;
        navControl.response.p2Offset = 3.24;
    }
    ROS_DEBUG("before bias_removal emit");
    emit bias_removal_returned(navControl);
}
