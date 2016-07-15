

#ifndef BASE_INTERFACE_HPP__
#define BASE_INTERFACE_HPP__


#include <ros/ros.h>

#include <boost/system/error_code.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#define NUM_BUFFERS_PER_PLUGIN 4

namespace base_classes
{
    enum interfaceType_t {Serial, UDP};

    class base_interface
    {

    public:
        //const boost::shared_ptr<boost::asio::io_service> interfaceService

        std::string pluginName;
        interfaceType_t interfaceType;
        bool interfaceStarted;

        //hw_interface will call to check if work can continue
        virtual bool interfaceReady() {}

        //called after io_service init
        virtual bool initPlugin(const boost::shared_ptr<boost::asio::io_service> ioService) {}

        //called after init, used to start interface and restart listen
        virtual bool startWork() {}

        //called to stop interface
        virtual bool stopWork() {}

        //plugin provided data handler that moves data into ROS
        virtual bool interfaceDataHandler() {}

        //called by hw_interface
        virtual bool verifyChecksum() {}

        virtual bool handleIORequest(const boost::system::error_code &ec, size_t bytesReceived) {}

        base_interface() {}
        virtual ~base_interface(){}

    protected:
        
        boost::shared_ptr<char> receivedData[NUM_BUFFERS_PER_PLUGIN];

    };
};

#endif //BASE_INTERFACE_HPP__
