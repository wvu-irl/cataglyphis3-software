

#ifndef BASE_SERIAL_INTERFACE_HPP__
#define BASE_SERIAL_INTERFACE_HPP__


#include <ros/ros.h>

#include <hw_interface/base_interface.hpp>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#include <boost/asio/basic_serial_port.hpp>
#include <boost/asio/serial_port.hpp>

#define MAX_SERIAL_READ 28

namespace base_classes
{
    class base_serial_interface : public base_interface
    {

    private:

        boost::shared_ptr<boost::asio::serial_port> interfacePort;

        bool interfaceReady();
        bool initPlugin(const boost::shared_ptr<boost::asio::io_service> ioService);
        bool startWork(); //probably set some flag and start listening on port
        bool stopWork();  //probably unset some flag

        //will receive request, then run plugin readHandler(), check work flag,
            //if work flag restart async read

    protected:

        std::string deviceName;

        base_serial_interface();

        //needs better name
        //this function calls the plugin's function to read in ROS params,
        //subscribe to topics, publish topics. This function should fill
        //in the protected member's info
        virtual bool subPluginInit() = 0;

        template<typename Option>
        boost::system::error_code setOption(const Option * newOption)
        {
            boost::system::error_code ec;
            if(interfacePort.get()){
                return interfacePort->set_option<Option>(*newOption, ec);
            }
            ec.assign(static_cast<int>(boost::system::errc::no_such_device), boost::asio::error::get_system_category());
            return ec;
        }

        template<typename Option>
        Option getOption(){
            Option returnValue;
            if(interfacePort.get()){
                interfacePort->get_option<Option>(returnValue);
            }
            return returnValue;
        }

    public:
        bool handleIORequest(const boost::system::error_code &ec, size_t bytesReceived);
        std::size_t completionCheck(const boost::system::error_code& error,
                                    std::size_t bytesReceived);

    };
};


#endif //BASE_SERIAL_INTERFACE_HPP__
