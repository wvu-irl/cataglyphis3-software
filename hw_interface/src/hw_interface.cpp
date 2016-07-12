#include <ros/ros.h>

#include <hw_interface/hw_interface.hpp>

#include <iostream>

#define THREAD_ID_TO_C_STR boost::lexical_cast<std::string>(boost::this_thread::get_id()).c_str()

hw_interface::hw_interface()
{

    addInterfacePlugins();

    interfaceService = boost::shared_ptr<boost::asio::io_service>
                                    (new boost::asio::io_service);

    interfaceWork = boost::shared_ptr<boost::asio::io_service::work>
                                    (new boost::asio::io_service::work(*interfaceService));
    initThreadPool();
    initInterfacePlugins();
}


//needs to be finished
//MUST CALL initInterfacePlugins AFTER io_service creation, not before!!!!
void hw_interface::addInterfacePlugins()
{
    pluginlib::ClassLoader<base_classes::base_interface> pluginLoader("hw_interface", "base_classes::base_interface");

    try {

        interfacePluginVector.push_back(pluginLoader.createInstance("<A ROS PARAM GOES HERE>")); //ros param with name of derived class
        if(!interfacePluginVector.back().get()) //if the last added plugin is valid
        {
            ROS_ERROR("The plugin failed to instantiate");
            interfacePluginVector.pop_back();
        }
    }
    catch(pluginlib::PluginlibException& ex)
    {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }
}

void hw_interface::initThreadPool()
{
    for(int i = 0; i < interfacePluginVector.size(); i++)
    {
        interfaceWorkerGroup.create_thread(boost::bind(&interface_worker::worker, interfaceService));
    }
}

void hw_interface::initInterfacePlugins()
{
    for(std::vector<boost::shared_ptr<base_classes::base_interface> >::iterator vectorIterator = interfacePluginVector.begin();
            vectorIterator != interfacePluginVector.end();
            vectorIterator++)
    {
        vectorIterator->get()->initPlugin();
    }
}

bool hw_interface::startInterfaces()
{
    for(std::vector<boost::shared_ptr<base_classes::base_interface> >::iterator vectorIterator = interfacePluginVector.begin();
            vectorIterator != interfacePluginVector.end();
            vectorIterator++)
    {
        vectorIterator->get()->startWork();
    }
}

bool hw_interface::stopInterfaces()
{
    for(std::vector<boost::shared_ptr<base_classes::base_interface> >::iterator vectorIterator = interfacePluginVector.begin();
            vectorIterator != interfacePluginVector.end();
            vectorIterator++)
    {
        vectorIterator->get()->stopWork();
    }
}

void interface_worker::worker(boost::shared_ptr<boost::asio::io_service> ioService)
{
    ROS_DEBUG("Thread <%s>:: Started Worker", THREAD_ID_TO_C_STR);


    //should change to poll if we want to profile individual thread performance
    while(!ioService->stopped())
    {
        try{
            ioService->run();
        }
        catch(const boost::system::error_code &ec)
        {
            ROS_ERROR("Exception Caught! \r\n %s", ec.message().c_str());
        }
        catch(const std::exception &ex)
        {
            ROS_ERROR("STD Exception Caught! \r\n %s", ex.what());
        }
    }

    ROS_DEBUG("Thread <%s>:: Stopping Worker", THREAD_ID_TO_C_STR);
}
