#include "voice.h"

Voice::Voice()
{
	voiceSayPhrasePublisher = nh.advertise<messages::VoiceSayPhrase>("/hsm/voicenode/voicesayphrase", 1000);
}

void Voice::call(std::string voiceString)
{
	voiceSayPhraseMsg.phrase = voiceString;
	voiceSayPhrasePublisher.publish(voiceSayPhraseMsg);
}
