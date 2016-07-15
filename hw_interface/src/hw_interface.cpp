#include <ros/ros.h>


#include <iostream>

#include <hw_interface/hw_interface.hpp>



#define THREAD_ID_TO_C_STR boost::lexical_cast<std::string>(boost::this_thread::get_id()).c_str()
#define NUM_THREADS_PER_PLUGIN 5

hw_interface::hw_interface() :
    pluginLoader("hw_interface", "base_classes::base_interface")
{

    ROS_DEBUG("Loading Plugins");
    addInterfacePlugins();
    ROS_DEBUG("Creating ASIO Services");
    interfaceService = boost::shared_ptr<boost::asio::io_service>
                                    (new boost::asio::io_service);
    ROS_DEBUG("Generating Work");
    interfaceWork = boost::shared_ptr<boost::asio::io_service::work>
                                    (new boost::asio::io_service::work(*interfaceService));
    ROS_DEBUG("Starting Thread Pool");
    initThreadPool();
    ROS_DEBUG("Initilizing Plugins");
    initInterfacePlugins();
    ROS_DEBUG("Starting Interfaces");
    startInterfaces();
}

hw_interface::~hw_interface()
{
    std::cout<<("Stopping Work, Stopping ASIO Services")<<std::endl;
    interfaceWork.reset();
    interfaceService->stop();
    interfaceService->reset();
    for(std::vector<boost::shared_ptr<base_classes::base_interface> >::iterator vectorIterator = interfacePluginVector.begin();
            vectorIterator != interfacePluginVector.end();
            vectorIterator++)
    {
        std::printf("Destroying Plugin: %s\r\n", vectorIterator->get()->pluginName.c_str());
        vectorIterator->reset();
    }
}


//needs to be finished
//MUST CALL initInterfacePlugins AFTER io_service creation, not before!!!!
void hw_interface::addInterfacePlugins()
{
    try {

        interfacePluginVector.push_back(pluginLoader.createInstance("hw_interface_plugins::netburner_UDP")); //ros param with name of derived class
        if(!interfacePluginVector.back().get()) //if the last added plugin is valid
        {
            ROS_ERROR("The plugin failed to instantiate");
            interfacePluginVector.pop_back();
        }
        else
        {
            ROS_INFO("Instatiated Interface Plugin %s", interfacePluginVector.back()->pluginName.c_str());
        }
    }
    catch(pluginlib::PluginlibException& ex)
    {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }
}

void hw_interface::initThreadPool()
{
    ROS_INFO("Starting Thread Pool");
    for(int i = 0; i < interfacePluginVector.size() * NUM_THREADS_PER_PLUGIN; i++)
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
        ROS_INFO("Initilizing Plugin: %s", vectorIterator->get()->pluginName.c_str());
        vectorIterator->get()->initPlugin(interfaceService);
    }
}

bool hw_interface::startInterfaces()
{
    for(std::vector<boost::shared_ptr<base_classes::base_interface> >::iterator vectorIterator = interfacePluginVector.begin();
            vectorIterator != interfacePluginVector.end();
            vectorIterator++)
    {
        ROS_INFO("Starting Plugin Work: %s", vectorIterator->get()->pluginName.c_str());
        bool returnValue = vectorIterator->get()->startWork();
        ROS_INFO("Plugin Start return %d", returnValue);
    }
}

bool hw_interface::stopInterfaces()
{
    for(std::vector<boost::shared_ptr<base_classes::base_interface> >::iterator vectorIterator = interfacePluginVector.begin();
            vectorIterator != interfacePluginVector.end();
            vectorIterator++)
    {
        ROS_INFO("Stopping Plugin Work: %s", vectorIterator->get()->pluginName.c_str());
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
        ROS_INFO("Thread <%s>:: Waiting for Work", THREAD_ID_TO_C_STR);
        ros::Duration pause(1);
        pause.sleep();
    }

    std::printf("Thread <%s>:: Stopping Worker\r\n", THREAD_ID_TO_C_STR);
}
