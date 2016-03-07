#ifndef COUNTER_H
#define COUNTER_H


class Counter
{
private:
	// Members
	int count;
	int overflow;
	int saved_count;
	int saved_overflow;

public:
	// Methods
	Counter(); // Constructor
	void increment();
	void reset();
	void operator++(void);
	void operator++(int);
	int operator()(void);
	void save();
	int delta();
};

#endif /* COUNTER_H */
