#include <ros/ros.h>

#include <hw_interface/base_UDP_interface.hpp>

#define THREAD_ID_TO_C_STR boost::lexical_cast<std::string>(boost::this_thread::get_id()).c_str()

base_classes::base_UDP_interface::base_UDP_interface()
{
    ROS_DEBUG("UDP Plugin initialized");
    interfaceType = base_classes::UDP;
    interfaceStarted = false;

    receivedData = boost::shared_ptr<char>(new char[UDP_FRAME_BUFFER_SIZE]);
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

bool base_classes::base_UDP_interface::initPlugin(const boost::shared_ptr<boost::asio::io_service> ioService)
{
    //call plugin setup
    subPluginInit();

    remoteEndpoint = boost::shared_ptr<boost::asio::ip::udp::endpoint>(new boost::asio::ip::udp::endpoint(remoteAddress, remotePort));
    //create the local endpoint
    localEndpoint = boost::shared_ptr<boost::asio::ip::udp::endpoint>(new boost::asio::ip::udp::endpoint(localAddress, localPort));
    //open, bind, and connect to the local endpoint
    interfaceSocket = boost::shared_ptr<boost::asio::ip::udp::socket>(new boost::asio::ip::udp::socket(*ioService, *localEndpoint));

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

    interfaceSocket.get()->async_receive(boost::asio::buffer(receivedData.get(), UDP_FRAME_BUFFER_SIZE),
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
        return true;
    }
    return false;
}

bool base_classes::base_UDP_interface::handleIORequest(const boost::system::error_code &ec, size_t bytesReceived)
{
    ROS_INFO("Thread <%s>:: Received Packet!", THREAD_ID_TO_C_STR);
    return startWork();
}
