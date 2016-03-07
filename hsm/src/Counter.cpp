#include "Counter.h"
#include <stdio.h>

Counter::Counter() // Constructor
{
	this->reset();
}

void Counter::increment()
{
	count++;
	if(count==0) {overflow++;}
}

void Counter::reset()
{
	count=0;
	overflow=0;
	saved_count=0;
	saved_overflow=0;
}

void Counter::operator++(void)
{
	this->increment();
}

void Counter::operator++(int i)
{
	this->increment();
}

int Counter::operator()(void)
{
	return count;
}

void Counter::save()
{
	saved_count = count;
	saved_overflow = overflow;
}

int Counter::delta()
{
	return (count-saved_count); //need to add overflow logic
}
