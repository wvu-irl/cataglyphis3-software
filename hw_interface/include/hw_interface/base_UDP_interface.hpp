

#ifndef BASE_UDP_INTERFACE_HPP__
#define BASE_UDP_INTERFACE_HPP__


#include <ros/ros.h>

#include <hw_interface/base_interface.hpp>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#define UDP_FRAME_BUFFER_SIZE 1500

namespace base_classes
{
    class base_UDP_interface : public base_interface
    {
        //friend class hw_interface;

    private:

        boost::shared_ptr<boost::asio::ip::udp::socket> interfaceSocket;
        boost::shared_ptr<boost::asio::ip::udp::endpoint> localEndpoint;
        boost::shared_ptr<boost::asio::ip::udp::endpoint> remoteEndpoint;

        bool interfaceReady();
        bool initPlugin(ros::NodeHandlePtr nhPtr,
                            const boost::shared_ptr<boost::asio::io_service> ioService);
        bool startWork(); //probably set some flag and start listening on port
        bool stopWork();  //probably unset some flag

        //will receive request, then run plugin readHandler(), check work flag,
            //if work flag restart async read

    protected:

        base_UDP_interface();

        //needs better name
        //this function calls the plugin's function to read in ROS params,
        //subscribe to topics, publish topics. This function should fill
        //in the protected member's info
        virtual bool subPluginInit(ros::NodeHandlePtr nhPtr) = 0;

        boost::asio::ip::address localAddress;
        int localPort;
        boost::asio::ip::address remoteAddress;
        int remotePort;

        //plugin provided data handler that moves data into ROS
        virtual bool interfaceReadHandler(const long &bufferSize, int arrayStartPos) = 0;

    public:
        bool handleIORequest(const boost::system::error_code &ec, size_t bytesReceived);

    };
};
#endif //BASE_UDP_INTERFACE_HPP__
