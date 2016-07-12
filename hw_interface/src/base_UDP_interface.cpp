#include <ros/ros.h>

#include <hw_interface/base_UDP_interface.hpp>


base_classes::base_UDP_interface::base_UDP_interface()
{
    interfaceType = base_classes::UDP;
    interfaceReady = false;
};

bool base_classes::base_UDP_interface::startWork()
{
    return true;
}

bool base_classes::base_UDP_interface::stopWork()
{
    return true;
}
