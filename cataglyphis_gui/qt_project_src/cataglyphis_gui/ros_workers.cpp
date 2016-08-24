 #include <ros_workers.h>

ros_workers::ros_workers()
{
    nh.reset(new ros::NodeHandle());
    _implSetup();
}

ros_workers::ros_workers(boost::shared_ptr<ros::NodeHandle> nhArg)
{
    nh = nhArg;
    _implSetup();
}

void ros_workers::_implSetup()
{
    qRegisterMetaType<messages::NavFilterControl>("messages::NavFilterControl");
    //qRegisterMetaType<messages::HSMSetNorthAngle>("messages::HSMSetNorthAngle");
    qRegisterMetaType<robot_control::RegionsOfInterest>("robot_control::RegionsOfInterest");
    qRegisterMetaType<messages::GlobalMapFull>("messages::GlobalMapFull");
    qRegisterMetaType<messages::NavFilterOut>("messages::NavFilterOut");
    qRegisterMetaType<messages::RobotPose>("messages::RobotPose");
    qRegisterMetaType<map_viewer_enums::mapViewerLayers_t>("map_viewer_enums::mapViewerLayers_t");
    qRegisterMetaType<messages::SetStartingPlatform>("messages::SetStartingPlatform");
    qRegisterMetaType<messages::ExecInfo>("messages::ExecInfo");
    qRegisterMetaType<robot_control::ModifyROI>("robot_control::ModifyROI");
    qRegisterMetaType<robot_control::ROI>("robot_control::ROI");

    navControlClient = nh->serviceClient<messages::NavFilterControl>("/navigation/navigationfilter/control");
    //hsmNAControlClient = nh->serviceClient<messages::HSMSetNorthAngle>("/hsm/masterexec/setnorthangle");
    mapManagerROIClient = nh->serviceClient<robot_control::RegionsOfInterest>("/control/mapmanager/regionsofinterest");
    mapManagerGlobalMapClient = nh->serviceClient<messages::GlobalMapFull>("/control/mapmanager/globalmapfull");

    navInfoTime = ros::Time::now();
    navInfoSubStarted = false;
    navInfoSub = ros::Subscriber();

    hsmGlobalPoseTime = ros::Time::now();
    hsmGlobalPoseSubStarted = false;
    hsmGlobalPosSub = ros::Subscriber();

    execInfoTime = ros::Time::now();
    execInfoSubStarted = false;
    execInfoSub = ros::Subscriber();
}

void ros_workers::on_run_set_starting_platform_service(messages::SetStartingPlatform serviceRequest)
{
    bool successful = serviceCall<messages::SetStartingPlatform>("/control/mapmanager/setstartingplatform", &serviceRequest);
    emit map_manager_set_starting_platform_service_returned(serviceRequest, successful);
}

void ros_workers::on_run_nav_service(messages::NavFilterControl serviceRequest)
{
    bool successful = serviceCall<messages::NavFilterControl>("/navigation/navigationfilter/control", &serviceRequest);
    emit nav_service_returned(serviceRequest, successful);
}

void ros_workers::on_run_modify_roi(robot_control::ModifyROI serviceRequest)
{
    bool successful = serviceCall<robot_control::ModifyROI>("/control/mapmanager/modifyroi", &serviceRequest);
    emit modify_roi_service_returned(serviceRequest, successful);
}

template<typename T>
bool ros_workers::serviceCall(const char *serviceName, T *serviceRequest)
{
    bool wasSuccessful = false;
    ros::ServiceClient serviceClient = nh->serviceClient<T>(serviceName);
    if(serviceClient.exists())
    {
        ROS_DEBUG("ros_workers::%s Exists", serviceName);
        if(serviceClient.call(*serviceRequest))
        {
            ROS_DEBUG("ros_workers::%s Success!", serviceName);
            wasSuccessful = true;
        }
        else
        {
            ROS_WARN("ros_workers:: %s failure!", serviceName);
        }
    }
    else
    {
        ROS_WARN("ros_workers::%s Does not Exist!", serviceName);
        ROS_WARN("ros_worker:: sleeping %d seconds", ON_SERIVCE_FAILURE_RETURN_PAUSE);
        ros::Duration pause(ON_SERIVCE_FAILURE_RETURN_PAUSE);
        pause.sleep();
    }
    return wasSuccessful;
}

void ros_workers::on_run_map_manager_global_map_request(map_viewer_enums::mapViewerLayers_t requestedLayer)
{
    bool wasSucessful = false;
    if(mapManagerGlobalMapClient.exists())
    {
        ROS_DEBUG("ros_workers::run Global Map Service:: ROI Service Exists!");
        if(mapManagerGlobalMapClient.call(lastGlobalMapMsg))
        {
            ROS_DEBUG("ros_workers:: run Global Map service:: ROI service call success!");
            wasSucessful = true;
        }
        else
        {
            ROS_WARN("ros_workers:: run Global Map Service:: ROI service call failure");
        }
    }
    else
    {
        ROS_WARN("ros_workers::run Global Map Service:: ROI Service Does not Exist!");
        ROS_WARN("ros_worker:: sleeping %d seconds", ON_SERIVCE_FAILURE_RETURN_PAUSE);
        ros::Duration pause(ON_SERIVCE_FAILURE_RETURN_PAUSE);
        pause.sleep();
    }
    emit map_manager_global_map_service_returned(lastGlobalMapMsg, requestedLayer, wasSucessful);
}



void ros_workers::on_run_map_manager_ROI_service()
{
    bool wasSucessful = false;
    if(mapManagerROIClient.exists())
    {
        ROS_DEBUG("ros_workers::run_ROI Service:: ROI Service Exists!");
        if(mapManagerROIClient.call(lastROIMsg))
        {
            ROS_DEBUG("ros_workers:: run ROI service:: ROI service call success!");
            wasSucessful = true;
        }
        else
        {
            ROS_WARN("ros_workers:: run ROI Service:: ROI service call failure");
        }
    }
    else
    {
        ROS_WARN("ros_workers::run ROI Service:: ROI Service Does not Exist!");
        ROS_WARN("ros_worker:: sleeping %d seconds", ON_SERIVCE_FAILURE_RETURN_PAUSE);
        ros::Duration pause(ON_SERIVCE_FAILURE_RETURN_PAUSE);
        pause.sleep();
    }
    emit map_manager_ROI_service_returned(lastROIMsg, wasSucessful);
}

//void ros_workers::on_run_nav_init_service(messages::NavFilterControl serviceRequest)
//{
//    bool wasSucessful = false;
//    messages::NavFilterControl navControl = serviceRequest;

//    if(navControlClient.exists())
//    {
//        ROS_DEBUG("ros_workers::nav_init_service:: Nav Service Exists!");
//        navControl.request.runBiasRemoval = false;
//        if(navControlClient.call(navControl.request, navControl.response))
//        {
//            ROS_DEBUG("ros_workers::nav_init_service:: Nav Service Call Sucess");
//            wasSucessful = true;
//        }
//        else
//        {
//            ROS_WARN("ros_workers::nav_init_service:: Nav Service Call Failure!");
//        }
//    }
//    else
//    {
//        ROS_WARN("ros_workers::nav_init_service:: Nav Service Does not Exist!");
//        ROS_WARN("ros_worker:: sleeping %d seconds", ON_SERIVCE_FAILURE_RETURN_PAUSE);
//        ros::Duration pause(ON_SERIVCE_FAILURE_RETURN_PAUSE);
//        pause.sleep();
//    }
//    if(hsmNAControlClient.exists())
//    {
//        lastHSMNAMsg.request.northAngle = navControl.request.northAngle;
//        if(hsmNAControlClient.call(lastHSMNAMsg.request, lastHSMNAMsg.response))
//        {
//            ROS_DEBUG("ros_workers::nav_init_service:: HSM Service Call Sucess");
//            wasSucessful = true;
//        }
//        else
//        {
//            ROS_WARN("ros_workers::nav_init_service:: HSM Service Call Failure!");
//        }
//    }
//    else
//    {
//        ROS_WARN("ros_workers::nav_init_service:: HSM Service Does not Exist!");
//    }
//    emit nav_init_returned(navControl, wasSucessful);
//}

void ros_workers::on_run_bias_removal_service()
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

void ros_workers::on_run_start_dead_reckoning_service()
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
    if(ros::Time::now().toSec() - navInfoTime.toSec() > NAV_INFO_MIN_PUB_TIME)
    {
        navInfoTime = ros::Time::now();
        emit nav_info_callback(this->lastNavMsg);
    }
}

void ros_workers::getHSMGlobalPoseCallback(const messages::RobotPose::ConstPtr &msg)
{
    this->lastHSMGlobalPoseMsg = *msg;
    if(ros::Time::now().toSec() - hsmGlobalPoseTime.toSec() > HSM_POSE_MIN_PUB_TIME)
    {
        hsmGlobalPoseTime = ros::Time::now();
        emit hsm_global_pose_callback(this->lastHSMGlobalPoseMsg);
    }
}

void ros_workers::getExecInfoCallback(const messages::ExecInfo::ConstPtr &msg)
{
    this->lastExecInfoMsg = *msg;
    if(ros::Time::now().toSec() - execInfoTime.toSec() > EXEC_INFO_MIN_PUB_TIME)
    {
        execInfoTime = ros::Time::now();
        emit exec_info_callback(this->lastExecInfoMsg);
    }
}

void ros_workers::on_run_exec_info_subscriber_start()
{
    ROS_DEBUG("ros_workers:: Entered exec info subscriber start");
    if(!execInfoSubStarted)
    {
        execInfoSubStarted = true;
        ROS_DEBUG("ros_workers::Starting New subscriber");
        this->execInfoSub = nh->subscribe("control/exec/info", 1,
                                                        &ros_workers::getExecInfoCallback, this);
    }
}

void ros_workers::on_run_exec_info_subscriber_stop()
{
    ROS_DEBUG("ros_workers:: Entered exec info subscriber stop");
    if(execInfoSubStarted)
    {
        ROS_DEBUG("ros_workers::Stopping exec Info subscriber");
        execInfoSubStarted = false;
        this->execInfoSub.shutdown();
    }
}

void ros_workers::on_run_nav_info_subscriber_start()
{
    ROS_DEBUG("ros_workers:: Entered nav info subscriber start");
    if(!navInfoSubStarted)
    {
        navInfoSubStarted = true;
        ROS_DEBUG("ros_workers::Starting New subscriber");
        this->navInfoSub = nh->subscribe("navigation/navigationfilterout/navigationfilterout", 1,
                                                        &ros_workers::getNavInfoCallback, this);
    }
}

void ros_workers::on_run_nav_info_subscriber_stop()
{
    ROS_DEBUG("ros_workers:: Entered nav info subscriber stop");
    if(navInfoSubStarted)
    {
        ROS_DEBUG("ros_workers::Stopping Nav Info subscriber");
        navInfoSubStarted = false;
        this->navInfoSub.shutdown();
    }
}

void ros_workers::on_run_hsm_global_pose_subscriber_start()
{
    ROS_DEBUG("ros_workers:: Entered hsm global pose subscriber start");
    if(!hsmGlobalPoseSubStarted)
    {
        hsmGlobalPoseSubStarted = true;
        ROS_DEBUG("ros_workers::Starting New subscriber hsm global pose");
        this->hsmGlobalPosSub = nh->subscribe("/hsm/masterexec/globalpose", 1,
                                                        &ros_workers::getHSMGlobalPoseCallback, this);
    }
}

void ros_workers::on_run_hsm_global_pose_subscriber_stop()
{
    ROS_DEBUG("ros_workers:: Entered hsm global pose subscriber stop");
    if(hsmGlobalPoseSubStarted)
    {
        ROS_DEBUG("ros_workers::Stopping Nav Info subscriber hsm global pose");
        hsmGlobalPoseSubStarted = false;
        this->hsmGlobalPosSub.shutdown();
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
