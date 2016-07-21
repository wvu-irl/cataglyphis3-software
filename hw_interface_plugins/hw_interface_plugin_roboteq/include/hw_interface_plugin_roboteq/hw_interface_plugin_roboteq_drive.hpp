#ifndef HW_INTERFACE_PLUGIN_ROBOTEQ_DRIVE_HPP__
#define HW_INTERFACE_PLUGIN_ROBOTEQ_DRIVE_HPP__

#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>

#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq.hpp>

#include <messages/ActuatorOut.h>
#include <messages/encoder_data.h>

namespace hw_interface_plugin_roboteq {

    class roboteq_drive : public hw_interface_plugin_roboteq::roboteq_serial
    {
        public:
            roboteq_drive(){}

            void rosMsgCallback(const messages::ActuatorOut::ConstPtr &msgIn);

        protected:
            messages::encoder_data lastMsgPub;

            bool implInit();
            bool implStart();
            bool implStop();
    };

}

PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_roboteq::roboteq_drive, base_classes::base_interface)

#endif //HW_INTERFACE_PLUGIN_ROBOTEQ_DRIVE_HPP__
