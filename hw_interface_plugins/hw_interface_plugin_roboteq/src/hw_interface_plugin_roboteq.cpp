#include <ros/ros.h>

#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq.hpp>


hw_interface_plugin_roboteq::roboteq_serial::roboteq_serial()
{
    ROS_INFO_EXTRA_SINGLE("Roboteq Plugin Instantiated");
    enableMetrics();
}

bool hw_interface_plugin_roboteq::roboteq_serial::subPluginInit()
{
    ROS_INFO_EXTRA_SINGLE("Roboteq Plugin Init");

    deviceName = "";

    ros::param::get(pluginName+"/deviceName", deviceName);
    int tempBaudRate = 0;
    ros::param::get(pluginName+"/baudrate", tempBaudRate);

    setOption<boost::asio::serial_port_base::baud_rate>(
                new boost::asio::serial_port_base::baud_rate(tempBaudRate));

    ROS_INFO("%s :: Device: %s :: Baudrate %d", pluginName.c_str(), deviceName.c_str(), tempBaudRate);
    return true;
}

bool hw_interface_plugin_roboteq::roboteq_serial::interfaceDataHandler()
{
    ROS_INFO_EXTRA_SINGLE("Roboteq Plugin Data Handler");
    return true;
}

bool hw_interface_plugin_roboteq::roboteq_serial::verifyChecksum()
{
    return true;
}
