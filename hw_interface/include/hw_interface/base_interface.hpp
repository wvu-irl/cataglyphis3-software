

#ifndef BASE_INTERFACE_HPP__
#define BASE_INTERFACE_HPP__


#include <ros/ros.h>

#include <boost/system/error_code.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <hw_interface/hw_interface.hpp>

namespace base_classes
{
    enum interfaceType_t {Serial, UDP};

    class base_interface
    {
        friend class hw_interface;


    public:
        //const boost::shared_ptr<boost::asio::io_service> interfaceService

        interfaceType_t interfaceType;

        virtual bool initPlugin();

        virtual bool startWork();

        virtual bool stopWork();

        virtual void readHandler();

        virtual ~base_interface(){}

    protected:
        base_interface();

        virtual void handleIORequest(const boost::system::error_code &ec, size_t bytesReceived);

    };
};

#endif //BASE_INTERFACE_HPP__
