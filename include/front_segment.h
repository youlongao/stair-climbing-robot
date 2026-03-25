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
	bool liftFrontToStep();		// control the front section to raise onto the step
	bool placeFrontOnStep();	// place the front wheel stably on the step surface
	void stopFrontSegment();	// stop all actions; used for phace completion, falut handing, or emergency stop

private:
	IDriveSection& drive_section_;	// control front wheels to move forward or stop
	IFrontDistanceSensor& front_distance_sensor_;	// used to detected the relative distance to the stairs
	ILinearAxis* front_lift_axis_;	// used to raise front section
	std::function<bool()> surface_confirmed_;	// step surface confrimation callback
};
}

#endif
