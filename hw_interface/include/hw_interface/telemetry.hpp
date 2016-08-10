
#ifndef TELEMETRY_TIME_HPP__
#define TELEMETRY_TIME_HPP__

#include <ros/ros.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

class delta_loop_time
{
    private:

        bool metricsEnabled;

        //boost::mutex metricMuxtex;
        //ros::Time does not need a thread lock because it is actually locked internally
        boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_mean> > acc;
        ros::Time lastTimeMetric;
        double deltaTime;

    public:
        delta_loop_time():
        lastTimeMetric(ros::Time::now().toNSec()),
        acc(boost::accumulators::tag::rolling_window::window_size = 150)
        {
            metricsEnabled = true;
        }
        bool enableMetrics()
        {
            metricsEnabled = true;
            this->lastTimeMetric = ros::Time::now();
            return metricsEnabled;
        };

        bool disableMetrics()
        {
            metricsEnabled = false;
            return metricsEnabled;
        };

        float updateMetrics()
        {
            if(metricsEnabled)
            {
                boost::scoped_ptr<char> output(new char[255]);
                deltaTime = ros::Time::now().toSec()-this->lastTimeMetric.toSec();
                acc(deltaTime);
                this->lastTimeMetric = ros::Time::now();
                return 1/(boost::accumulators::rolling_mean(acc));
            }
            return 0;
        };

        std::string printMetrics(bool printSideEffect)
        {
            if(metricsEnabled)
            {
                boost::scoped_ptr<char> output(new char[255]);
                deltaTime = ros::Time::now().toSec()-this->lastTimeMetric.toSec();
                acc(deltaTime);
                sprintf(output.get(), "Delta Time %2.3f:: Hz %2.2f:: Avg Hz %2.2f",
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
        };
};

#endif //TELEMETRY_TIME_HPP__

