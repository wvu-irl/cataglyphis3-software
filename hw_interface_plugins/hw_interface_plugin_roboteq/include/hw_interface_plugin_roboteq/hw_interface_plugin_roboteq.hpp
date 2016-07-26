#ifndef HW_INTERFACE_PLUGIN_ROBOTEQ_HPP__
#define HW_INTERFACE_PLUGIN_ROBOTEQ_HPP__

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <hw_interface/shared_const_buffer.hpp>
#include <hw_interface/base_interface.hpp>
#include <hw_interface/base_serial_interface.hpp>

#include <messages/ActuatorOut.h>
#include <messages/encoder_data.h>
#include <messages/GrabberFeedback.h>

namespace hw_interface_plugin_roboteq {

    enum controller_t {Other, Left_Drive_Roboteq, Right_Drive_Roboteq, Grabber};

    class roboteq_serial : public base_classes::base_serial_interface
    {
    public:
        typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> matcherIterator;

        roboteq_serial();
        virtual ~roboteq_serial() {} //need to implement closing of the port here

    protected:
        ros::NodeHandlePtr nh;

        messages::ActuatorOut latestActuatorCmd;
        controller_t roboteqType;

        bool subPluginInit(ros::NodeHandlePtr nhPtr);
        void setInterfaceOptions();
        bool interfaceReadHandler(const long &length, int arrayStartPos);
        bool verifyChecksum();

        virtual bool implInit() = 0;
        virtual bool implStart() = 0;
        virtual bool implStop() = 0;
        virtual bool implDataHandler(const long &bufferSize, int arrayStartPos) = 0;
        virtual void rosMsgCallback(const messages::ActuatorOut::ConstPtr &msgIn) = 0;

        //since the function prototype is different from the required one for ASIO, we must use boost::bind
        //to reassign parameters and create a functor that we can pass into ASIO.
        std::pair<matcherIterator, bool> matchFooter(matcherIterator begin, matcherIterator end, const char *sequence);
    };
}


#endif //HW_INTERFACE_PLUGIN_ROBOTEQ_HPP__
