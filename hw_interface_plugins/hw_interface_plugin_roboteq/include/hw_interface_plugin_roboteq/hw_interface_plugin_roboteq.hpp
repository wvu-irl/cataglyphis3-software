#ifndef HW_INTERFACE_PLUGIN_ROBOTEQ_HPP__
#define HW_INTERFACE_PLUGIN_ROBOTEQ_HPP__

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <hw_interface/base_interface.hpp>
#include <hw_interface/base_serial_interface.hpp>



namespace hw_interface_plugin_roboteq {

    class roboteq_serial : public base_classes::base_serial_interface
    {
    public:
        roboteq_serial();
        virtual ~roboteq_serial() {} //need to implement closing of the port here

    protected:
        bool subPluginInit();
        bool interfaceDataHandler();
        bool verifyChecksum();
    };
}



PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_roboteq::roboteq_serial, base_classes::base_interface)



#endif //HW_INTERFACE_PLUGIN_ROBOTEQ_HPP__
