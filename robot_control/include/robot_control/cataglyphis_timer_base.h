#ifndef CATAGLYPHIS_TIMER_BASE_H
#define CATAGLYPHIS_TIMER_BASE_H

class CataglyphisTimerBase
{
public:
	// Methods
	virtual void setPeriod(float periodIn) = 0; // sec
	virtual void adjustPeriod(float periodAdjust) = 0; // sec
	virtual void start() = 0;
	virtual void stop() = 0;
	virtual void pause() = 0;
	virtual void resume() = 0;
	// Members
	float period;
	double startTime;
	double pauseTime;
	bool paused;
	bool running;
};

#endif // CATAGLYPHIS_TIMER_BASE_H
