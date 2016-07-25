#include <ros/ros.h>

#include <hw_interface/base_UDP_interface.hpp>

#define THREAD_ID_TO_C_STR boost::lexical_cast<std::string>(boost::this_thread::get_id()).c_str()

base_classes::base_UDP_interface::base_UDP_interface()
{
    ROS_DEBUG_EXTRA_SINGLE("UDP Plugin initialized");
    interfaceType = base_classes::UDP;
    interfaceStarted = false;

    receivedData = boost::shared_array<uint8_t>(new uint8_t[500]);
    remoteEndpoint = boost::shared_ptr<boost::asio::ip::udp::endpoint>();
    localEndpoint = boost::shared_ptr<boost::asio::ip::udp::endpoint>();
    interfaceSocket = boost::shared_ptr<boost::asio::ip::udp::socket>();
}

bool base_classes::base_UDP_interface::interfaceReady()
{
    if(!interfaceSocket.get())
    {
        return false;
    }
    return interfaceSocket->is_open();
}

bool base_classes::base_UDP_interface::initPlugin(ros::NodeHandlePtr nhPtr,
                                                    const boost::shared_ptr<boost::asio::io_service> ioService)
{
    //call plugin setup
    ROS_DEBUG_EXTRA_SINGLE("Calling Plugin's Init");
    subPluginInit(nhPtr);

    ROS_DEBUG_EXTRA_SINGLE("Creating Endpoints");
    remoteEndpoint = boost::shared_ptr<boost::asio::ip::udp::endpoint>(new boost::asio::ip::udp::endpoint(remoteAddress, remotePort));
    //create the local endpoint
    localEndpoint = boost::shared_ptr<boost::asio::ip::udp::endpoint>(new boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), localPort));
    //open, bind, and connect to the local endpoint
    ROS_DEBUG_EXTRA_SINGLE("Opening Local Socket");
    interfaceSocket = boost::shared_ptr<boost::asio::ip::udp::socket>(new boost::asio::ip::udp::socket(*ioService, *localEndpoint));
    ROS_DEBUG_EXTRA_SINGLE("Socket Created");
    return interfaceReady();
}

bool base_classes::base_UDP_interface::startWork()
{
    if(!interfaceReady())
    {
        return false;
    }

    if(!interfaceStarted)
    {
        interfaceStarted = true;
    }

    interfaceSocket.get()->async_receive(boost::asio::buffer(receivedData.get(), 100),
                                            boost::bind(&base_UDP_interface::handleIORequest, this,
                                                            boost::asio::placeholders::error(),
                                                            boost::asio::placeholders::bytes_transferred()));
    return true;
}

bool base_classes::base_UDP_interface::stopWork()
{
    if(interfaceStarted)
    {
        //free all queued work
        interfaceSocket->cancel();
        //probably shouldn't close
        interfaceSocket->close();

        interfaceStarted = false;
        return !interfaceSocket->is_open();
    }
    return false;
}

bool base_classes::base_UDP_interface::handleIORequest(const boost::system::error_code &ec, size_t bytesReceived)
{
    printMetrics(true);
    ROS_INFO("Thread <%s>:: %s:: Received Packet!:: Size %lu", THREAD_ID_TO_C_STR, this->pluginName.c_str(), bytesReceived);

    //call plugin's data handler
    if(!interfaceReadHandler(bytesReceived, dataArrayStart))
    {
        ROS_ERROR("Error Occurred in plugin data Handler <%s>", this->pluginName.c_str());
    }

    //restart the work
    return startWork();
}
