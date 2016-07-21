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

uint16_t base_classes::base_interface::calcCRC16Block(const void * const buf, std::size_t numOfBytes)
{
    return boost::crc<16, 0x1021, 0xFFFF, 0, false, false>(buf, numOfBytes);
}

//this should be multi access safe, as members are generated on the stack each call.
//need to double check
uint32_t base_classes::base_interface::calcCRC32Block(const void * const buf, std::size_t numOfBytes)
{
    return boost::crc<32, 0x1021, 0xFFFF, 0, false, false>(buf, numOfBytes);
}

std::size_t base_classes::base_interface::streamMatcherDelimAndLength(const boost::system::error_code &error, long totalBytesInBuffer,
                                                                      const int packetLengthInBytes, const char *headerSequence,
                                                                      const char *footerSequence, void *dataStartPosPtr)
{
    ROS_DEBUG("%s:: Length and footer matcher %lu", pluginName.c_str(), totalBytesInBuffer);
    if(totalBytesInBuffer <= std::strlen(footerSequence))
    {
        ROS_DEBUG("%s:: Matcher Returning Early", pluginName.c_str());
        return packetLengthInBytes - totalBytesInBuffer;
    }
    if((packetLengthInBytes - totalBytesInBuffer) <= 0)
    {
        ROS_DEBUG("%s:: Full Length Packet Received", pluginName.c_str());
        const int footerLength = std::strlen(footerSequence);
        int i = 0;
        int j = footerLength-1;
        for(i = 0; i < footerLength; i++)
        {
            if(receivedData[ totalBytesInBuffer - 1 - i ] != footerSequence[j])
            {
                //THIS IS WHERE AN HSM invalid message can be sent
                std::printf("\r\n");
                ROS_ERROR("%s:: Invalid Footer\r\n", pluginName.c_str());
                return footerLength;
            }
            j--;
        }
        const int headerLength = std::strlen(headerSequence);
        for(i = 0; i < headerLength; i++)
        {
            if(receivedData[totalBytesInBuffer - packetLengthInBytes + i] != headerSequence[i])
            {
                //THIS IS WHERE AN HSM invalid message can be sent
                std::printf("\r\n");
                ROS_ERROR("%s:: Invalid Header\r\n", pluginName.c_str());
                return packetLengthInBytes;
            }
        }
        //should post something to HSM here.

        ROS_DEBUG("%s:: Header Found, Footer Found, Correct Length, Good Packet", pluginName.c_str());
        dataStartPosPtr = (void*)( receivedData.get() + totalBytesInBuffer - packetLengthInBytes );
        return 0;
    }
    return packetLengthInBytes - totalBytesInBuffer;
}
