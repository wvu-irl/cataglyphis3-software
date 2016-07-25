#include <ros/ros.h>

#include <hw_interface_plugin_roboteq//hw_interface_plugin_roboteq_grabber.hpp>



void hw_interface_plugin_roboteq::roboteq_grabber::rosMsgCallback(const messages::ActuatorOut::ConstPtr &msgIn)
{

}

bool hw_interface_plugin_roboteq::roboteq_grabber::implInit()
{
    std::string tempString;
    if(ros::param::get(pluginName+"/subscribeToTopic", tempString))
    {
        rosDataSub = nh->subscribe(tempString, 1, &roboteq_grabber::rosMsgCallback, this);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic subscription name", pluginName.c_str());
    }

    if(ros::param::get(pluginName+"/publishToTopic", tempString))
    {
        rosDataPub = nh->advertise<messages::GrabberFeedback>(tempString, 1, false);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic advertisment name", pluginName.c_str());
    }

//need to start async timers here for grabber monitoring
    return true;
}

bool hw_interface_plugin_roboteq::roboteq_grabber::implStart()
{

    return true;
}

bool hw_interface_plugin_roboteq::roboteq_grabber::implStop()
{

    return true;
}

bool hw_interface_plugin_roboteq::roboteq_grabber::implDataHandler(const long &bufferSize,
                                                                        int arrayStartPos)
{

    return true;
}
