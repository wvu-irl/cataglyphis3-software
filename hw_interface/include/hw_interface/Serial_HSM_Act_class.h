#ifndef SERIAL_HSM_ACT_CLASS_H
#define SERIAL_HSM_ACT_CLASS_H
#include <hsm/HSM_Action_Sub_Superclass.h>

class Serial_HSM_Act: public HSM_Action_Sub
{
	public:
	// Members
	bool* publish_enable_ptr;
	// Methods
	Serial_HSM_Act(bool* publish_enable_ptr_input); // Constructor
	//void actionCallback(const hsm::HSM_Action::ConstPtr& msg_in);
};

#endif /* SERIAL_HSM_ACT_CLASS_H */
