

#ifndef HW_INTERFACE_HPP__
#define HW_INTERFACE_HPP__


#include <ros/ros.h>
#include <hw_interface/base_interface.hpp>
#include <hw_interface/base_serial_interface.hpp>
#include <hw_interface/base_UDP_interface.hpp>
#include <pluginlib/class_loader.h>

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#define NUM_THREADS_PER_PLUGIN 2

namespace interface_worker
{
    void worker(boost::shared_ptr<boost::asio::io_service> ioService);
};

class hw_interface {
    //friend class interface_worker;

private:
    ros::NodeHandlePtr node;

    boost::shared_ptr<boost::asio::io_service> interfaceService;
    boost::shared_ptr<boost::asio::io_service::work> interfaceWork;

    std::vector<boost::shared_ptr<base_classes::base_interface> > interfacePluginVector;

    boost::thread_group interfaceWorkerGroup;

    pluginlib::ClassLoader<base_classes::base_interface> pluginLoader;

    bool initPlugin(boost::shared_ptr<base_classes::base_interface> pluginPtr,
                        std::string pluginName);

    void initInterfacePlugins();
    void initThreadPool();

public:


    hw_interface();
    hw_interface(ros::NodeHandlePtr nhArg);
    //hw_interface(int maxNumOfThreads);

    virtual ~hw_interface();

    void addInterfacePlugins();

    bool startInterfaces();
    bool stopInterfaces();      //once node has stopped, reset interfaceWork ptr.




};


#endif //HW_INTERFACE_HPP__
