#include <ros/ros.h>

#include <hw_interface/base_serial_interface.hpp>

base_classes::base_serial_interface::base_serial_interface()
{
    ROS_DEBUG_EXTRA_SINGLE("Serial Plugin Initialized");
    interfaceType = base_classes::Serial;
    interfaceStarted = false;

    enableMetrics();

    receivedData = boost::shared_ptr<char>(new char[500]);

    interfacePort = boost::shared_ptr<boost::asio::serial_port>();


}

bool base_classes::base_serial_interface::interfaceReady()
{
    if(!interfacePort.get())
    {
        return false;
    }
    return interfacePort->is_open();
}

bool base_classes::base_serial_interface::initPlugin(const boost::shared_ptr<boost::asio::io_service> ioService)
{
    //to use the base classes option settings, a blank port must be provided before
    //initilizing plugin
    interfacePort = boost::shared_ptr<boost::asio::serial_port>(new boost::asio::serial_port(*ioService));

    //call plugin setup
    ROS_DEBUG_EXTRA_SINGLE("Calling Plugin's Init");
    subPluginInit();

    interfacePort->open(deviceName);

    return interfaceReady();
}

bool base_classes::base_serial_interface::startWork()
{
    if(!interfaceReady())
    {
        return false;
    }
    if(!interfaceStarted)
    {
        interfaceStarted = true;
    }

    boost::asio::async_read(*interfacePort, boost::asio::buffer(receivedData.get(), MAX_SERIAL_READ),
//                                    boost::bind(&base_serial_interface::completionCheck,this,
//                                                    boost::asio::placeholders::error(),
//                                                    boost::asio::placeholders::bytes_transferred()),
                                    boost::bind(&base_serial_interface::handleIORequest,this,
                                                    boost::asio::placeholders::error(),
                                                    boost::asio::placeholders::bytes_transferred()));
    return true;
}

bool base_classes::base_serial_interface::stopWork()
{
    if(interfaceStarted)
    {
        interfacePort->cancel();
        interfacePort->close();
        interfaceStarted = false;
        return !interfacePort->is_open();
    }
    return false;
}

bool base_classes::base_serial_interface::handleIORequest(const boost::system::error_code &ec, size_t bytesReceived)
{
    printMetrics(true);
    ROS_INFO("Thread <%s>:: %s:: Received Packet!:: Size %lu", THREAD_ID_TO_C_STR, this->pluginName.c_str(), bytesReceived);

    //call plugin's data handler
    if(!interfaceDataHandler())
    {
        ROS_ERROR("Error Occurred in plugin data Handler <%s>", this->pluginName.c_str());
    }

    //restart the work
    return startWork();
}

std::size_t base_classes::base_serial_interface::completionCheck(const boost::system::error_code &error, std::size_t bytesReceived)
{
    return MAX_SERIAL_READ-bytesReceived;
}


