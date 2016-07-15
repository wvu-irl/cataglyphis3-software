#ifndef HW_INTERFACE_PLUGIN_NB_HPP__
#define HW_INTERFACE_PLUGIN_NB_HPP__

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <hw_interface/base_interface.hpp>
#include <hw_interface/base_UDP_interface.hpp>



namespace hw_interface_plugins {

    class netburner_UDP : public base_classes::base_UDP_interface
    {
    public:
        netburner_UDP();
        virtual ~netburner_UDP() {}

    protected:
        bool subPluginInit();
        bool interfaceDataHandler();
        bool verifyChecksum();
    };

}


PLUGINLIB_EXPORT_CLASS(hw_interface_plugins::netburner_UDP, base_classes::base_interface)



#endif //HW_INTERFACE_PLUGIN_NB_HPP__
