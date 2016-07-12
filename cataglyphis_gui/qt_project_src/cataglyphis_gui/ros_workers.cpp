#include <ros_workers.h>

ros_workers::ros_workers(boost::shared_ptr<ros::NodeHandle> nhArg)
{
    nh = nhArg;
    navControlClient = nh->serviceClient<messages::NavFilterControl>("/nav/nav_filter_control_service");
    hsmNAControlClient = nh->serviceClient<messages::HSMSetNorthAngle>("/hsm/masterexec/setnorthangle");
    navInfoTime = ros::Time::now();
    navInfoSubStarted = false;
    navInfoCallbackSub = ros::Subscriber();
    //workerMutex = boost::shared_ptr<QMutex>(new QMutex);
}

void ros_workers::run_nav_service(messages::NavFilterControl serviceRequest,
                                    const int callerID)
{
    bool wasSucessful = false;
    if(navControlClient.exists())
    {
        ROS_DEBUG("ros_workers::run_nav_service:: Nav Service Exists!");
        if(navControlClient.call(serviceRequest))
        {
            ROS_DEBUG("ros_workers::run_nav_service:: Nav Service Call Sucess");
            wasSucessful = true;
        }
        else
        {
            ROS_WARN("ros_workers::run_nav_service:: Nav Service Call Failure!");
        }
    }
    else
    {
        ROS_WARN("ros_workers::run_nav_service:: Nav Service Does not Exist!");
        ROS_WARN("ros_worker:: sleeping %d seconds", ON_SERIVCE_FAILURE_RETURN_PAUSE);
        ros::Duration pause(ON_SERIVCE_FAILURE_RETURN_PAUSE);
        pause.sleep();
    }
    emit nav_service_returned(serviceRequest, wasSucessful, callerID);
}

void ros_workers::run_nav_init_service(messages::NavFilterControl serviceRequest)
{
    bool wasSucessful = false;
    messages::NavFilterControl navControl = serviceRequest;

    if(navControlClient.exists())
    {
        ROS_DEBUG("ros_workers::nav_init_service:: Nav Service Exists!");
        navControl.request.runBiasRemoval = false;
        if(navControlClient.call(navControl.request, navControl.response))
        {
            ROS_DEBUG("ros_workers::nav_init_service:: Nav Service Call Sucess");
            wasSucessful = true;
        }
        else
        {
            ROS_WARN("ros_workers::nav_init_service:: Nav Service Call Failure!");
        }
    }
    else
    {
        ROS_WARN("ros_workers::nav_init_service:: Nav Service Does not Exist!");
        ROS_WARN("ros_worker:: sleeping %d seconds", ON_SERIVCE_FAILURE_RETURN_PAUSE);
        ros::Duration pause(ON_SERIVCE_FAILURE_RETURN_PAUSE);
        pause.sleep();
    }
    if(hsmNAControlClient.exists())
    {
        lastHSMNAMsg.request.northAngle = navControl.request.northAngle;
        if(hsmNAControlClient.call(lastHSMNAMsg.request, lastHSMNAMsg.response))
        {
            ROS_DEBUG("ros_workers::nav_init_service:: HSM Service Call Sucess");
            wasSucessful = true;
        }
        else
        {
            ROS_WARN("ros_workers::nav_init_service:: HSM Service Call Failure!");
        }
    }
    else
    {
        ROS_WARN("ros_workers::nav_init_service:: HSM Service Does not Exist!");
    }
    emit nav_init_returned(navControl, wasSucessful);
}

void ros_workers::run_bias_removal_service()
{
    ROS_DEBUG("ros_workers:: Bias RemovalService Handler");
    messages::NavFilterControl navControl;
    bool wasSucessful = false;

    if(navControlClient.exists())
    {
        ROS_DEBUG("ros_workers::bias_removal_service:: Nav Service Exists!");
        navControl.request.runBiasRemoval = true;
        if(navControlClient.call(navControl.request, navControl.response))
        {
            ROS_DEBUG("ros_workers::bias_removal_service:: Nav Service Call Sucess");
            wasSucessful = true;
        }
        else
        {
            ROS_WARN("ros_workers::bias_removal_service:: Nav Service Call Failure!");
        }
    }
    else
    {
        ROS_WARN("ros_workers::bias_removal_service:: Nav Service Does not Exist!");
        ROS_WARN("ros_worker:: sleeping %d seconds", ON_SERIVCE_FAILURE_RETURN_PAUSE);
        ros::Duration pause(ON_SERIVCE_FAILURE_RETURN_PAUSE);
        pause.sleep();
        navControl.response.p1Offset = NAN;
        navControl.response.q1Offset = NAN;
        navControl.response.r1Offset = NAN;
        navControl.response.p2Offset = NAN;
        navControl.response.q2Offset = NAN;
        navControl.response.r2Offset = NAN;
        navControl.response.p3Offset = NAN;
        navControl.response.q3Offset = NAN;
        navControl.response.r3Offset = NAN;
    }
    ROS_DEBUG("ros_workers:: before bias_removal emit");
    emit bias_removal_returned(navControl, wasSucessful);
}

void ros_workers::run_start_dead_reckoning_service()
{
    ROS_DEBUG("ros_workers:: starting dead reckoning");
    messages::NavFilterControl navControl;
    bool wasSucessful;

    if(navControlClient.exists())
    {
        ROS_DEBUG("ros_workers::dead_reckoning_service:: Nav Service Exists!");
        navControl.request.runBiasRemoval = false;
        navControl.request.beginForkliftDeadReckoning = true;
        if(navControlClient.call(navControl.request, navControl.response))
        {
            ROS_DEBUG("ros_workers::dead_reckoning_service:: Nav Service Call Sucess");
            wasSucessful = false;
        }
        else
        {
            ROS_WARN("ros_workers::dead_reckoning_service:: Nav Service Call Failure!");
        }
    }
    else
    {
        ROS_WARN("ros_workers::dead_reckoning_service:: Nav Service Does not Exist!");
        ROS_WARN("ros_worker:: sleeping %d seconds", ON_SERIVCE_FAILURE_RETURN_PAUSE);
        ros::Duration pause(ON_SERIVCE_FAILURE_RETURN_PAUSE);
        pause.sleep();
        navControl.response.p1Offset = NAN;
        navControl.response.q1Offset = NAN;
        navControl.response.r1Offset = NAN;
        navControl.response.p2Offset = NAN;
        navControl.response.q2Offset = NAN;
        navControl.response.r2Offset = NAN;
        navControl.response.p3Offset = NAN;
        navControl.response.q3Offset = NAN;
        navControl.response.r3Offset = NAN;
    }
    ROS_DEBUG("ros_workers::dead_reckoning_service call finished");
    emit dead_reckoning_service_returned(navControl, wasSucessful);
}

void ros_workers::getNavInfoCallback(const messages::NavFilterOut::ConstPtr &msg)
{
    this->lastNavMsg = *msg;
    //rate limit amount of signals sent to the QT event queue
    if(navInfoTime.toSec() - ros::Time::now().toSec() > NAV_INFO_MIN_PUB_TIME)
    {
        navInfoTime = ros::Time::now();
        emit nav_info_callback(this->lastNavMsg);
    }
}

void ros_workers::run_nav_info_subscriber_start()
{
    ROS_DEBUG("ros_workers:: Entered nav info subscriber start");
    if(!navInfoSubStarted)
    {
        navInfoSubStarted = true;
        ROS_DEBUG("ros_workers::Starting New subscriber");
        this->navInfoCallbackSub = nh->subscribe("navigation/navigationfilterout/navigationfilterout", 1,
                                                        &ros_workers::getNavInfoCallback, this);
    }
}

void ros_workers::run_nav_info_subscriber_stop()
{
    ROS_DEBUG("ros_workers:: Entered nav info subscriber stop");
    if(navInfoSubStarted)
    {
        ROS_DEBUG("ros_workers::Stopping Nav Info subscriber");
        navInfoSubStarted = false;
        this->navInfoCallbackSub.shutdown();
    }
}


//not working


//void ros_workers::run_service(const QString serviceName,
//                                ros::SerializedMessage &request,
//                                ros::SerializedMessage &response)
//{
//    bool wasSucessful = false;
//    ROS_DEBUG("ros_workers::run_service:: calling service %s", serviceName.toStdString().c_str());

//    //ros::service::call()
//    if(ros::service::exists(serviceName.toStdString().c_str(), true))
//    {
//        ROS_DEBUG("ros_workers::run_service:: Service %s Exists", serviceName.toStdString().c_str());
//        if(ros::service::call(serviceName.toStdString().c_str(),
//                                request, response))
//        {
//            ROS_DEBUG("ros_workers::run_service:: Service Call to %s Sucessful", serviceName.toStdString().c_str());
//            wasSucessful = true;
//        }
//        else
//        {
//            ROS_DEBUG("ros_workers::run_service:: Service Call to %s failed", serviceName.toStdString().c_str());
//        }
//    }
//    else
//    {
//        ROS_DEBUG("ros_workers::run_service:: Service %s does not exist!", serviceName.toStdString().c_str());
//        ROS_DEBUG("ros_worker::run_service sleeping %d seconds", ON_SERIVCE_FAILURE_RETURN_PAUSE);
//        ros::Duration pause(ON_SERIVCE_FAILURE_RETURN_PAUSE);
//        pause.sleep();
//    }
//    emit nav_service_returned(wasSucessful);
//}
