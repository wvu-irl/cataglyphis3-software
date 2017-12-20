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
#include <messages/VoiceSayPhrase.h>
namespace messages_package = messages;
#include <festival/festival.h>

void voiceSayPhraseCallback(const messages_package::VoiceSayPhrase::ConstPtr &msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "voice_node");
	ROS_INFO("voice_node running...");
	ros::NodeHandle nh;
	int heap_size = 210000;  // default scheme heap size
	int load_init_files = 1; // we want the festival init files loaded
	ros::Subscriber voiceSayPhraseSub = nh.subscribe<messages_package::VoiceSayPhrase>("/hsm/voicenode/voicesayphrase", 1000, voiceSayPhraseCallback);

	festival_initialize(load_init_files,heap_size);
	festival_eval_command("(voice_kal_diphone)"); //kal, don, rad, kdl

	festival_say_text("Hello.. I am Cataglyphis. This is the last time.");

	ros::spin();

	return 0;
}

void voiceSayPhraseCallback(const messages_package::VoiceSayPhrase::ConstPtr &msg)
{
	festival_say_text(msg->phrase.c_str());
}
