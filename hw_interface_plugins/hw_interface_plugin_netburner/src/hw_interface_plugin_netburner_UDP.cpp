#include <ros/ros.h>

#include <hw_interface_plugin_netburner/hw_interface_plugin_netburner_UDP.hpp>


hw_interface_plugin_netburner::netburner_UDP::netburner_UDP()
{
    ROS_INFO_EXTRA_SINGLE("NB Plugin Init");
    //localAddress = boost::asio::ip::address::from_string("192.168.2.122");


    enableMetrics();
}

bool hw_interface_plugin_netburner::netburner_UDP::subPluginInit()
{

    ROS_INFO_EXTRA("%s Plugin instantiated", pluginName.c_str());
    std::string tempAddress = "";
    if(!ros::param::get(pluginName+"/remoteAddress", tempAddress))
    {
        ROS_WARN_EXTRA_SINGLE("failed to find remote address param");
    }


    remoteAddress = boost::asio::ip::address::from_string(tempAddress.c_str());
    localPort = 0;
    remotePort = 0;

    ros::param::get(pluginName+"/remotePort", remotePort);
    ros::param::get(pluginName+"/localPort", localPort);

    ROS_INFO("%s :: Remote: %s :: Remote Port: %d :: Local Port: %d", pluginName.c_str(), tempAddress.c_str(),
                                                                        remotePort, localPort);

    return true;
}

bool hw_interface_plugin_netburner::netburner_UDP::interfaceDataHandler()
{
    ROS_INFO_EXTRA_SINGLE("NB Plugin Data Handler");
    return true;
}

bool hw_interface_plugin_netburner::netburner_UDP::verifyChecksum()
{
    return true;
}
