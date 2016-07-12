#ifndef PORT_SUPERCLASS_H
#define PORT_SUPERCLASS_H

class Port
{
public:
	// Members
	
	// Methods
	virtual bool port_read()=0;
	virtual bool port_write()=0;
};

#endif /* PORT_SUPERCLASS_H */
