

#ifndef BASE_SERIAL_INTERFACE_HPP__
#define BASE_SERIAL_INTERFACE_HPP__


#include <ros/ros.h>

#include <hw_interface/base_interface.hpp>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#include <boost/asio/basic_serial_port.hpp>
#include <boost/asio/serial_port.hpp>

#define MAX_SERIAL_READ 250

namespace base_classes
{
    class base_serial_interface : public base_interface
    {

    private:

        bool interfaceReady();
        bool initPlugin(ros::NodeHandlePtr nhPtr,
                            const boost::shared_ptr<boost::asio::io_service> ioService);
        bool startWork(); //probably set some flag and start listening on port
        bool stopWork();  //probably unset some flag

        //will receive request, then run plugin readHandler(), check work flag,
            //if work flag restart async read


        //this ptr is used to pass the beginning of data to the plugin data handler.
        //  its void because we don't type discriminate around here.
        //But mainly so the plugin can interpret the data however it wants to.

    protected:
        boost::shared_ptr<boost::asio::serial_port> interfacePort;

        std::string deviceName;
        boost::asio::streambuf interfaceDataBuffer;

        int readLength;
        std::string headerString, footerString;

        base_serial_interface();

        //needs better name
        //this function calls the plugin's function to read in ROS params,
        //subscribe to topics, publish topics. This function should fill
        //in the protected member's info
        virtual bool subPluginInit(ros::NodeHandlePtr nhPtr) = 0;
        virtual void setInterfaceOptions() = 0;

        virtual bool interfaceReadHandler(const long &bufferSize, int arrayStartPos) = 0;

        void interfaceWriteHandler(const hw_interface_support_types::shared_const_buffer &buffer);
        void postInterfaceWriteRequest(const hw_interface_support_types::shared_const_buffer &buffer);

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


        //this definition is used to check if a certain character sequence has been encountered
        //on the stream. The begin and end iterators represent positions in the stream that this
        //current functor is working. The return iterator represents where the next function call
        //(if needed) will begin in the buffer stream. The return bool indicates if the ASIO Service
        //is complete and should call the plugin IO Handler. (true = yes, call handler)

        //Plugins should overide this function if it intends on using this functionality

        virtual std::pair<matcherIterator, bool> matchFooter(matcherIterator begin, matcherIterator end, const char* sequence)
        {
            return std::make_pair(end, true);
        }



    };
};


#endif //BASE_SERIAL_INTERFACE_HPP__
