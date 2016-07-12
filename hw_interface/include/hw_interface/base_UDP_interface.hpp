

#ifndef BASE_UDP_INTERFACE_HPP__
#define BASE_UDP_INTERFACE_HPP__


#include <ros/ros.h>

#include <hw_interface/hw_interface.hpp>
#include <hw_interface/base_interface.hpp>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

namespace base_classes
{
    class base_UDP_interface : base_interface
    {
        friend class hw_interface;

    private:
        bool interfaceReady;
        boost::shared_ptr<boost::asio::ip::udp::socket> interfaceSocket;
        boost::shared_ptr<boost::asio::ip::udp::endpoint> socketEndpoint;

        bool startWork(); //probably set some flag and start listening on port
        bool stopWork();  //probably unset some flag

        //will receive request, then run plugin readHandler(), check work flag,
            //if work flag restart async read
        void handleIORequest(const boost::system::error_code &ec, size_t bytesReceived);

    protected:

        base_UDP_interface();




    public:

    };
};
#endif //BASE_UDP_INTERFACE_HPP__
