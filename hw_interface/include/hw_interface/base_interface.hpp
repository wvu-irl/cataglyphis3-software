

#ifndef BASE_INTERFACE_HPP__
#define BASE_INTERFACE_HPP__


#include <ros/ros.h>

#include <boost/system/error_code.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <boost/filesystem/path.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#define THREAD_ID_TO_C_STR boost::lexical_cast<std::string>(boost::this_thread::get_id()).c_str()
#define FILENAME_C_STR boost::filesystem::path(__FILE__).filename().c_str()
#define LINENUMBER_C_STR boost::lexical_cast<std::string>(__LINE__).c_str()
#define ROS_INFO_EXTRA_SINGLE(arg1  ) ROS_INFO("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR)
#define ROS_INFO_EXTRA(arg1, args...) ROS_INFO("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR, args)
#define ROS_ERROR_EXTRA_SINGLE(arg1  ) ROS_ERROR("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR)
#define ROS_ERROR_EXTRA(arg1, args...) ROS_ERROR("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR, args)
#define ROS_DEBUG_EXTRA_SINGLE(arg1  ) ROS_DEBUG("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR)
#define ROS_DEBUG_EXTRA(arg1, args...) ROS_DEBUG("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR, args)
#define ROS_WARN_EXTRA_SINGLE(arg1  ) ROS_WARN("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR)
#define ROS_WARN_EXTRA(arg1, args...) ROS_WARN("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR, args)
#define NUM_BUFFERS_PER_PLUGIN 4

namespace base_classes
{
    enum interfaceType_t {Serial, UDP, Custom};

    class base_interface
    {

    public:
        //const boost::shared_ptr<boost::asio::io_service> interfaceService

        std::string pluginName;
        interfaceType_t interfaceType;
        bool interfaceStarted;
        bool enabled;

        boost::shared_ptr<ros::Publisher> rosDataPub; //publisher, data from interface to ros
        boost::shared_ptr<ros::Subscriber> rosDataSub;//subscriber, data from ros to interface

        //hw_interface will call to check if work can continue
        virtual bool interfaceReady() {}

        //called after io_service init
        virtual bool initPlugin(const boost::shared_ptr<boost::asio::io_service> ioService) {}

        //called after init, used to start interface and restart listen
        virtual bool startWork() {}

        //called to stop interface
        virtual bool stopWork() {}

        //plugin provided data handler that moves data into ROS
        virtual bool interfaceDataHandler() {}

        //called by hw_interface
        virtual bool verifyChecksum() {}

        virtual bool handleIORequest(const boost::system::error_code &ec, size_t bytesReceived) {}

        bool enableMetrics();
        bool disableMetrics();
        std::string printMetrics(bool printSideEffect);

        base_interface() :
            lastTimeMetric(ros::Time::now().toNSec()),
            acc(boost::accumulators::tag::rolling_window::window_size = 300)
        {
            enabled = true;
        }
        virtual ~base_interface(){}

    protected:
        
        boost::shared_ptr<char> receivedData;

    private:

        bool metricsEnabled;

        //boost::mutex metricMuxtex;
        //ros::Time does not need a thread lock because it is actually locked internally
        boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_mean> > acc;
        ros::Time lastTimeMetric;
        double deltaTime;
    };
};

#endif //BASE_INTERFACE_HPP__
