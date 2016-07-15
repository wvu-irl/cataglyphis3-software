#include <ros/ros.h>

#include <hw_interface_plugin_netburner/hw_interface_plugin_netburner_UDP.hpp>


hw_interface_plugins::netburner_UDP::netburner_UDP()
{
    ROS_INFO("HW_Interface NB Plugin Initstatiated");
    pluginName = "NB1_UDP";
    localAddress = boost::asio::ip::address::from_string("192.168.2.143");
    remoteAddress = boost::asio::ip::address::from_string("192.168.2.122");
    localPort = 9003;
    remotePort = 9001;
}

bool hw_interface_plugins::netburner_UDP::subPluginInit()
{
    ROS_INFO("NB Plugin Init");
    return true;
}

bool hw_interface_plugins::netburner_UDP::interfaceDataHandler()
{
    ROS_INFO("NB Plugin Data Handler");
    return true;
}

bool hw_interface_plugins::netburner_UDP::verifyChecksum()
{
    return true;
}
