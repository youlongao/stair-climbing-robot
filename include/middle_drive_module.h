#ifndef MIDDLE_DRIVE_MODULE_H
#define MIDDLE_DRIVE_MODULE_H

#include <functional>

#include "config.h"
#include "hardware_interfaces.h"

namespace Robot
{
class MiddleDriveModule
{
public:
	/*
	drive_section: The underlying drive interface of the mid-section drive wheels, controling motor's move or stop
	middle_support_confirmed: a callback function, use to confirm wheher mid-section drive wheel has landed on the new support surface;
	*/
	MiddleDriveModule(IDriveSection& drive_section,
					  std::function<bool()> middle_support_confirmed = {});

	bool advanceToStep();	// control mid-section drive wheels move forward until the mid-section 
							// support successfully confirmed
	void driveForward();	// move forward
	void holdPosition();	// stop the movement of the middle drive wheels, keeping current state

private:
	IDriveSection& drive_section_;
	std::function<bool()> middle_support_confirmed_;
};
}

#endif
