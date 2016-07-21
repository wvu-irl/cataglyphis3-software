#include <ros/ros.h>

#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq_drive.hpp>

void hw_interface_plugin_roboteq::roboteq_drive::rosMsgCallback(const messages::ActuatorOut::ConstPtr &msgIn)
{

}

bool hw_interface_plugin_roboteq::roboteq_drive::implInit()
{
    std::string tempString;
    if(ros::param::get(pluginName+"/subscribeToTopic", tempString))
    {
        rosDataSub = nh->subscribe(tempString, 1, &roboteq_drive::rosMsgCallback, this);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic subscription name", pluginName.c_str());
    }

    if(ros::param::get(pluginName+"/publishToTopic", tempString))
    {
        rosDataPub = nh->advertise<messages::encoder_data>(tempString, 1, false);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic advertisment name", pluginName.c_str());
    }

//need to start async timers here for grabber monitoring
    return true;
}

bool hw_interface_plugin_roboteq::roboteq_drive::implStart()
{

    return true;
}

bool hw_interface_plugin_roboteq::roboteq_drive::implStop()
{

    return true;
}
