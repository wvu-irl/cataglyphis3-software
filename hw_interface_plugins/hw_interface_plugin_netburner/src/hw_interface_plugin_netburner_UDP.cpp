#include <ros/ros.h>

#include <hw_interface_plugin_netburner/hw_interface_plugin_netburner_UDP.hpp>


hw_interface_plugin_netburner::netburner_UDP::netburner_UDP()
{
    ROS_DEBUG_EXTRA_SINGLE("A Wild NB Plugin Appeared!");
    //localAddress = boost::asio::ip::address::from_string("192.168.2.122");

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
           ros::console::notifyLoggerLevelsChanged();
        }
    enableMetrics();
}

bool hw_interface_plugin_netburner::netburner_UDP::subPluginInit(ros::NodeHandlePtr nhPtr)
{
    ROS_DEBUG_EXTRA("%s Plugin Init", pluginName.c_str());

    std::string tempString = "";
    if(!ros::param::get(pluginName+"/remoteAddress", tempString))
    {
        ROS_WARN_EXTRA_SINGLE("failed to find remote address param");
    }
    else
    {
        remoteAddress = boost::asio::ip::address::from_string(tempString.c_str());
    }

    if(ros::param::get(pluginName+"/topicSubscription", tempString))
    {
        //rosDataSub = ros::NodeHandlePtr
    }

    localPort = 0;
    remotePort = 0;
    ros::param::get(pluginName+"/remotePort", remotePort);
    ros::param::get(pluginName+"/localPort", localPort);

    ROS_INFO("%s :: Remote IP: %s :: Remote Port: %d :: Local Port: %d",
                pluginName.c_str(), remoteAddress.to_string().c_str(), remotePort, localPort );

    return true;
}


bool hw_interface_plugin_netburner::netburner_UDP::interfaceReadHandler(const long &bufferSize, int arrayStartPos)
{
    ROS_INFO_EXTRA_SINGLE("NB Plugin Data Handler");
    return true;
}

bool hw_interface_plugin_netburner::netburner_UDP::verifyChecksum()
{
    return true;
}
