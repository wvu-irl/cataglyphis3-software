

#ifndef BASE_SERIAL_INTERFACE_HPP__
#define BASE_SERIAL_INTERFACE_HPP__


#include <ros/ros.h>

#include <hw_interface/hw_interface.hpp>
#include <hw_interface/base_interface.hpp>

namespace base_classes
{
    class base_serial_interface : base_interface
    {
        friend class hw_interface;

    private:


    protected:
        base_serial_interface()
        {
            //ROS_DEBUG("Init Serial Interface");
            base_interface::interfaceType = base_classes::Serial;
        }



    };
};


#endif //BASE_SERIAL_INTERFACE_HPP__
