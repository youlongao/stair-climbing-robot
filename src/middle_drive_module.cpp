#include "middle_drive_module.h"

#include <utility>

namespace Robot
{
MiddleDriveModule::MiddleDriveModule(IDriveSection& drive_section,
									 std::function<bool()> middle_support_confirmed)
	: drive_section_(drive_section),
	  middle_support_confirmed_(std::move(middle_support_confirmed))
{
}

// control mid-section drive wheels moving forward until the mid-section support is successfully confirmed
bool MiddleDriveModule::advanceToStep()
{
	// make mid-section drive wheels move forward at a slow speed in order to stablly aprroach new support surface
	drive_section_.setNormalizedSpeed(RobotConfig::Motion::CREEP_SPEED,
									  RobotConfig::Motion::CREEP_SPEED);
	
	// if mid-section support level is provided to confirm a callback,
	// and has confrimed mid-section drive wheels landing on the new support surface 
	if (middle_support_confirmed_ && middle_support_confirmed_())
	{
		holdPosition();
		return true;
	}

	return false;
}

// directly control mid-section drive wheel moving forward at a approach speed
void MiddleDriveModule::driveForward(const float speed)
{
	drive_section_.setNormalizedSpeed(speed, speed);
}

bool MiddleDriveModule::isSupportConfirmed() const
{
	return middle_support_confirmed_ && middle_support_confirmed_();
}

// keep current position
void MiddleDriveModule::holdPosition()
{
	drive_section_.brake();
}
}
