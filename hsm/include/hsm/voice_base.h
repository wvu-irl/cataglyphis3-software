#ifndef VOICE_BASE_H
#define VOICE_BASE_H
#include <string>

class VoiceBase
{
public:
	virtual void call(std::string voiceString) = 0;
};

#endif // VOICE_BASE_H
