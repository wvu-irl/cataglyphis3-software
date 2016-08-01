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

	festival_say_text("Hello.. I am Cataglyphis. I think, therefore I am.");

	ros::spin();

	return 0;
}

void voiceSayPhraseCallback(const messages_package::VoiceSayPhrase::ConstPtr &msg)
{
	festival_say_text(msg->phrase.c_str());
}
