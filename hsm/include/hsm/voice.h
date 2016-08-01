#ifndef VOICE_H
#define VOICE_H
#include <ros/ros.h>
#include <messages/VoiceSayPhrase.h>
#include "voice_base.h"

//#define VOICESAY(stringPhrase) {VoiceBase::voiceObj->call(stringPhrase);} // Need to #define voiceObj <obj> where this class is implemented

class Voice : public VoiceBase
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
