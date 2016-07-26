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

    std::string tempString = "";
    if(ros::param::get(pluginName+"/controllerType", tempString))
    {
        if(tempString.compare("Right_Drive")) { roboteqType = controller_t::Right_Drive_Roboteq; }
        else if(tempString.compare("Left_Drive")) { roboteqType = controller_t::Left_Drive_Roboteq; }
        else { roboteqType = controller_t::Other; }
    }
    else
    {
        ROS_ERROR("%s:: No Controller Type Specifiecompleted!", pluginName.c_str());
    }

    ROS_DEBUG("%s:: Implementation Init", pluginName.c_str());
    implInit();
    ROS_INFO_EXTRA_SINGLE("Roboteq Plugin Init");
    enableMetrics();
    setupStreamMatcherDelimAndLength(readLength, headerString.c_str(),
                                        footerString.c_str());
    deviceName = "";
    ros::param::get(pluginName+"/deviceName", deviceName);

    return true;
}

void hw_interface_plugin_roboteq::roboteq_serial::setInterfaceOptions()
{
	int tempBaudRate = 0;
    ros::param::get(pluginName+"/baudrate", tempBaudRate);
    setOption<boost::asio::serial_port_base::baud_rate>(
                new boost::asio::serial_port_base::baud_rate(tempBaudRate));
    //8 bits per character
    setOption<boost::asio::serial_port_base::character_size>(
    			new boost::asio::serial_port_base::character_size( 8 ));

    //flow control
    setOption<boost::asio::serial_port_base::flow_control>(
    			new boost::asio::serial_port_base::flow_control(
    										boost::asio::serial_port_base::flow_control::type::none));

    //parity
    setOption<boost::asio::serial_port_base::parity>(
    			new boost::asio::serial_port_base::parity(
    										boost::asio::serial_port_base::parity::type::none));

    //stop-bits
    setOption<boost::asio::serial_port_base::stop_bits>(
    			new boost::asio::serial_port_base::stop_bits(
    										boost::asio::serial_port_base::stop_bits::type::one));

    ROS_INFO("%s :: Device: %s :: Baudrate %d", pluginName.c_str(), deviceName.c_str(), tempBaudRate);
}

bool hw_interface_plugin_roboteq::roboteq_serial::interfaceReadHandler(const long &length,
                                                                            int arrayStartPos)
{
    ROS_INFO_EXTRA_SINGLE("Roboteq Plugin Data Handler");
    if(! implDataHandler(length, arrayStartPos))
    {
        ROS_ERROR("%s :: Implementation Data Handler returned a BAD Return", pluginName.c_str());
    }
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
