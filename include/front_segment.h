#ifndef FRONT_SEGMENT_H
#define FRONT_SEGMENT_H

#include <functional>

#include "config.h"
#include "hardware_interfaces.h"

namespace Robot
{
class FrontSegment
{
	// construct front segement objects
public:
	FrontSegment(IDriveSection& drive_section,	// front drive wheel control interface
				 IFrontDistanceSensor& front_distance_sensor,	// a forward sensor is used to determine whether the robot is approaching steps 
				 ILinearAxis* front_lift_axis = nullptr,	// front lifting shaft
				 std::function<bool()> surface_confirmed = {});		// callback function used to confirm whether the front wheels have landed on the step

	bool approachStep();	// control front segement to approach step
	float approachAssistSpeed() const;	// speed that the middle wheels should mirror during approach
	bool liftFrontToStep();		// control the front section to raise onto the step
	bool liftFrontUntilClearance();	// raise the front section while all drive wheels stay stopped
	bool placeFrontOnStep();	// place the front wheel stably on the step surface
	void driveForward(float speed);	// move the front drive wheels forward at a selected speed
	void stopDrive();	// stop only the front drive wheels
	void brakeDrive();	// actively brake only the front drive wheels
	bool isSurfaceConfirmed() const;	// true after the front downward sensor confirms landing
	void stopFrontSegment();	// stop all actions; used for phace completion, falut handing, or emergency stop

private:
	void logApproachStatus(float distance_m, float command_speed);

	IDriveSection& drive_section_;	// control front wheels to move forward or stop
	IFrontDistanceSensor& front_distance_sensor_;	// used to detected the relative distance to the stairs
	ILinearAxis* front_lift_axis_;	// used to raise front section
	std::function<bool()> surface_confirmed_;	// step surface confrimation callback
	int approach_close_sample_count_{0};
	float approach_assist_speed_{0.0F};
	Timestamp last_approach_log_time_{};
};
}

#endif
