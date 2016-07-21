#include <ros/ros.h>

#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq.hpp>


hw_interface_plugin_roboteq::roboteq_serial::roboteq_serial()
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
           ros::console::notifyLoggerLevelsChanged();
        }
    ROS_INFO_EXTRA_SINGLE("Roboteq Plugin Instantiated");
}

bool hw_interface_plugin_roboteq::roboteq_serial::subPluginInit(ros::NodeHandlePtr nhPtr)
{
    nh = ros::NodeHandlePtr(nhPtr);

    implInit();

    ROS_INFO_EXTRA_SINGLE("Roboteq Plugin Init");
    enableMetrics();

    setupStreamMatcherDelimAndLength(28, "Az", "P1fC", dataStartPositionPtr);

    deviceName = "";
    ros::param::get(pluginName+"/deviceName", deviceName);

    int tempBaudRate = 0;
    ros::param::get(pluginName+"/baudrate", tempBaudRate);

    setOption<boost::asio::serial_port_base::baud_rate>(
                new boost::asio::serial_port_base::baud_rate(tempBaudRate));

    ROS_INFO("%s :: Device: %s :: Baudrate %d", pluginName.c_str(), deviceName.c_str(), tempBaudRate);
    return true;
}

bool hw_interface_plugin_roboteq::roboteq_serial::interfaceDataHandler(const long &bufferSize, void* buf)
{
    ROS_DEBUG_EXTRA_SINGLE("Roboteq Plugin Data Handler");

    //for(int i = 0; )

    return true;
}

bool hw_interface_plugin_roboteq::roboteq_serial::verifyChecksum()
{
    return true;
}

typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> matcherIterator;

std::pair<matcherIterator, bool>
hw_interface_plugin_roboteq::roboteq_serial::matchFooter(matcherIterator begin, matcherIterator end, const char* sequence)
{
    int i = 0;
    for(matcherIterator footerIt = (end-std::strlen(sequence)); footerIt!=end; footerIt++)
    {
        if(*footerIt!=sequence[i])
        {
            return std::make_pair(begin, false);
        }
        i++;
    }
    return std::make_pair(begin, true);
}
