#include <ros/ros.h>

#include <hw_interface/base_interface.hpp>

#include <boost/accumulators/statistics/mean.hpp>

bool base_classes::base_interface::enableMetrics()
{
    metricsEnabled = true;
    this->lastTimeMetric = ros::Time::now();
    return metricsEnabled;
}

bool base_classes::base_interface::disableMetrics()
{
    metricsEnabled = false;
    return metricsEnabled;
}

std::string base_classes::base_interface::printMetrics(bool printSideEffect)
{
    if(metricsEnabled)
    {
        boost::scoped_ptr<char> output(new char[255]);
        deltaTime = ros::Time::now().toSec()-this->lastTimeMetric.toSec();
        acc(deltaTime);
        sprintf(output.get(), "Thread <%s>:: Interface <%15s>:: Delta Time %2.3f:: Hz %2.2f:: Avg Hz %2.2f",
                                        THREAD_ID_TO_C_STR, this->pluginName.c_str(),
                                        deltaTime, 1/deltaTime, 1/(boost::accumulators::rolling_mean(acc)));
        this->lastTimeMetric = ros::Time::now();
        if(printSideEffect)
        {
            ROS_DEBUG("%s", output.get());
        }

        std::string outputString(1, *output);
        return outputString;
    }
    else
    {
        return "";
    }
}
