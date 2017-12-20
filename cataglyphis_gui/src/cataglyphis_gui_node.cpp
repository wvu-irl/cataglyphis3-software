/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include "../../robot_control/include/robot_control/cataglyphis_timer.h"
#include <ros/timer.h>

#ifdef STATIC
    #include <QtPlugin>
//Q_IMPORT_PLUGIN(png)
#endif


#include <QApplication>
#include "core_app.h"

int main(int argc, char **argv)
{
//    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
//       ros::console::notifyLoggerLevelsChanged();
//    }
    Q_INIT_RESOURCE(resources);
    ros::init(argc, argv, "GUI_Node", ros::init_options::AnonymousName);
    ROS_INFO("GUI_Node - ros::init complete");
    boost::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());
    ROS_INFO("GUI_Node - node handle created");

    QApplication qCoreApp(argc, argv);
    core_app cataglyphis_gui(0, nh);
    cataglyphis_gui.show();

    return qCoreApp.exec();

    //bias testClass;

    //ROS_INFO("Timer is valid? %d", testClass.timer.getInternalTimer()->isValid());
    //ROS_INFO("TIMER has pending? %d", testClass.timer.getInternalTimer()->hasPending());
//    ROS_INFO("Starting Timer");
//    testClass.timer.setPeriod(1);
//    testClass.timer.start();

//    ros::Rate loopRate(2); //set loop rate to 50Hz

//    int counter = 0;
//    while(ros::ok())
//    {
//        ROS_INFO("Counter Value %d", counter);

//        if((counter!=0)&&(counter % 20) == 0)
//        {
//            ROS_WARN("PAUSING TIMER");
//            testClass.timer.pause();
//        }
//        if((counter!=0)&&(counter % 26) == 0)
//        {
//            ROS_WARN("RESUMING Timer");
//            testClass.timer.resume();
//        }
//        //ROS_INFO("Timer is valid? %d", testClass.timer.getInternalTimer()->isValid());
//        //ROS_INFO("TIMER has pending? %d", testClass.timer.getInternalTimer()->hasPending());
//        counter++;
//        ros::spinOnce();
//        loopRate.sleep();
//    }
}


//porlar grid from starting platform

//cartesian grid relative to map NA

//local map to disply NA changes with a submit button to HSM and NAV

//each sample candidate should display location, what CV thought type, and confidence

//needto add option to push wait to front of exec queue.
//need to add commit button to map display
//


//class bias
//{
//public:
//    bias():
//        timer(&bias::callback, this)
//    {
//        j = 0;
//    }

//    void callback(const ros::TimerEvent&)
//    {
//        ROS_WARN("Timer Callback");
//        timer.start();
//    }
//    CataglyphisTimer<bias> timer;

//private:
//    int j;
//};
