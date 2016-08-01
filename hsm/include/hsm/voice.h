#ifndef VOICE_H
#define VOICE_H
#include <ros/ros.h>
#include <messages/VoiceSayPhrase.h>

//#define VOICESAY(stringPhrase) voiceObj.call(stringPhrase);

class Voice
{
public:
	Voice();
	void call(std::string voiceString);
private:
	ros::NodeHandle nh;
	ros::Publisher voiceSayPhrasePublisher;
	messages::VoiceSayPhrase voiceSayPhraseMsg;
};

#endif // VOICE_H
